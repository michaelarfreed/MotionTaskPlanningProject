import utils
import settings
from hl_action import *
import handoff
import drawer_action
from rapprentice.PR2 import PR2, Arm, mirror_arm_joints
# try:
#   import openrave_input
# except:
#   print "Warning: ROS imports failed. Okay if not using ROS."


class ActionExecutor(object):
  def __init__(self, env, use_ros=False, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.use_ros = use_ros

    if self.use_ros:
      self.pr2 = PR2(self.robot)
      utils.set_rave_limits_to_soft_joint_limits(self.robot)
      # motion planner sometimes fails. slightly increase dof limits to resolve
      # lower, upper = self.robot.GetDOFLimits()
      # lower -= 0.01
      # upper += 0.01
      # self.robot.SetDOFLimits(lower, upper)
    else:
      self.pr2 = None
      if settings.USE_SOFT_LIMITS:
        utils.set_rave_limits_to_soft_joint_limits(self.robot)

      # start arms on the side
      left_joints = Arm.L_POSTURES['side2']
      right_joints = mirror_arm_joints(left_joints).tolist()
      self.robot.SetDOFValues(
        right_joints + left_joints,
        self.robot.GetManipulator("rightarm").GetArmIndices().tolist() +
        self.robot.GetManipulator("leftarm").GetArmIndices().tolist())

    self.unmovable_objects = unmovable_objects

    self.action_list = []
    self.saved_env_states = []  # the stored state at index n is the state of the environment before action n
    ##TODO[SS]: clean up: where is instantiation_generator set
    self.instantiation_generator = None
    self.resume_from_lineno = 0
    self.mp_completed_index = 0

    if not settings.run_test_mode[0]:
      raw_input("Press Enter to start!")

  def reset_all(self):
    self.action_list = []
    self.saved_env_states = []
    self.instantiation_generator = None
    self.resume_from_lineno = 0
    self.mp_completed_index = 0

  def prep_partial(self, resume_from=0):
    self.resume_from_lineno = resume_from
    self.action_list = self.action_list[:self.resume_from_lineno]
    self.instantiation_generator = None

  def save_openrave_state(self, n):
    if n < len(self.saved_env_states):
      self.saved_env_states[n] = EnvManager.save_openrave_state(self.env)
    elif n == len(self.saved_env_states):
      self.saved_env_states.append(EnvManager.save_openrave_state(self.env))
    else:
      raise Exception("Trying to save state with index {}, but \
        saved_env_states is only {} long!".format(n, len(self.saved_env_states)))

  def restore_openrave_state(self, n):
    # print 'RESETTING ENV TO STATE: {}'.format(n)
    with self.env:
      EnvManager.restore_openrave_state(self.env, self.saved_env_states[n])

  def reset_all_actions(self):
    self.reset_actions(self.action_list)

  def reset_actions(self, actions):
    for action in actions:
      action.reset()

  def execute_all(self):
    while True:
      self.restore_openrave_state(0)
      raw_input("Run in sim!")
      for action in self.action_list:
        action.execute_trajevents(sim_only=False)
      again = raw_input("Again? ([y]/n)")
      if again == 'n':
        break

    if self.pr2 is not None:
      self.restore_openrave_state(0)
      raw_input("Run on PR2!")
      for action in self.action_list:
        if settings.REPLAN:
          print "Replanning and executing: {}".format(action.get_name())
          action.replan()
        action.execute_trajevents(use_pr2=True)

  def get_next_instantiation(self):
    if self.instantiation_generator is None:
      self.instantiation_generator = self._try_refine()

    error = self.instantiation_generator.next()
    if error is not None:
      raise error

  def _try_refine(self):
    if settings.DO_BACKTRACKING:
      MAX_FAST_MP_DIFF = 100
    else:
      MAX_FAST_MP_DIFF = 0

    print "\r\nTrying to find error-free instantiation..."
    self.reset_actions(self.action_list[self.resume_from_lineno:])
    cur_index = self.resume_from_lineno

    if len(self.saved_env_states) == 0:
      self.save_openrave_state(cur_index)

    self.mp_completed_index = min(self.mp_completed_index, self.resume_from_lineno)
    n = 0
    MAX_ITERS = 100
    while cur_index < len(self.action_list):
      ##TODO[SS]: long monolithic code. break up into IK, MP and partial-traj generation modules
      self.restore_openrave_state(cur_index)
      try:
        n += 1
        if settings.USE_MAX_ITERATIONS:
          print "Iteration {} of {}.".format(n, MAX_ITERS)
        assert cur_index >= 0, "Error: current index <0"
        print "FAST TRYING: {0}: {1}".format(cur_index, 
                                             self.action_list[cur_index].get_name())
        if cur_index <= self.mp_completed_index:
          self.mp_completed_index = cur_index
          self.reset_actions(self.action_list[self.mp_completed_index+1:])
        self.action_list[cur_index].find_and_execute_next_fastevents()
        cur_index += 1
        self.save_openrave_state(cur_index)
      except StopIteration:
        if not settings.DO_BACKTRACKING:
          raise Exception("No backtracking; failed at step: {}".format(cur_index))
        if cur_index <= self.resume_from_lineno:
          break
        if settings.USE_MAX_ITERATIONS:
          if n >= MAX_ITERS:
            print "Max Iterations {} hit!".format(MAX_ITERS)
            break
        self.reset_actions(self.action_list[cur_index:])
        cur_index -= 1

      if cur_index - self.mp_completed_index >= MAX_FAST_MP_DIFF or\
         cur_index == len(self.action_list):
        print "CHECKING MOTION PLANS FROM {} to {} of {}...".format(
          self.mp_completed_index, cur_index - 1, len(self.action_list) - 1)
        try:
          self.restore_openrave_state(self.mp_completed_index)
          for index in range(self.mp_completed_index, cur_index):
            action = self.action_list[index]
            if action.last_trajevents is None:
              print "MOTION PLANNING: {0}: {1}".format(self.mp_completed_index, 
                                                       action.get_name())
              action.calc_trajevents()
              self.mp_completed_index += 1
            action.execute_trajevents()
          if self.mp_completed_index == len(self.action_list):
            yield None
        except StopIteration:
          print "MP FAILED AT: {}".format(self.mp_completed_index)
          if self.mp_completed_index == self.resume_from_lineno - 1:
            break
          cur_index = self.mp_completed_index
          self.reset_actions(self.action_list[cur_index+1:])

    print "\r\nNo error-free instantiation found!"
    self.reset_actions(self.action_list[self.resume_from_lineno:])
    cur_index = self.resume_from_lineno
    while cur_index < len(self.action_list):
      self.restore_openrave_state(cur_index)
      try:
        print "TRYING MP (error-free first): {0}: {1}".format(cur_index, self.action_list[cur_index].get_name())
        self.action_list[cur_index].find_next_trajevents()
        self.action_list[cur_index].execute_trajevents()
        self.mp_completed_index += 1
        cur_index += 1
        self.save_openrave_state(cur_index)
      except ActionError as e:
        if not settings.DO_BACKTRACKING:
          raise Exception("No backtracking; failed at step: {}".format(cur_index))  
        if "unreachable" in e.problem:
          if cur_index == self.resume_from_lineno:
            yield e
            # continue
            # if do something here is to be avoided, the
            # cur_index should be allowed to backtrack to 0
            # -- since a break always leads to do something here
            break
          self.reset_actions(self.action_list[cur_index:])
          cur_index -= 1
          self.mp_completed_index -= 1
        yield e
      except StopIteration:
        if not settings.DO_BACKTRACKING:
            raise Exception("No backtracking; failed at step: {}".format(cur_index))  
        if cur_index == self.resume_from_lineno:
          break
        self.reset_actions(self.action_list[cur_index:])
        cur_index -= 1
        self.mp_completed_index -= 1
    raise InstantiationExhaustedException("Do something here... instantiations exhausted")  # TODO???

  def _add_action(self, action):
    if action.lineno >= self.resume_from_lineno:
      if action.lineno == len(self.action_list):
        self.action_list.append(action)
      elif action.lineno < len(self.action_list):
        self.action_list[action.lineno] = action
      else:
        raise Exception("Bad action lineno: {}".format(action.lineno))

  def add_moveto_gp(self, lineno, obj, manip='rightarm', req_side=None, pose_name=None):
    self._add_action(MovetoGPAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, obj, manip, req_side, pose_name))

  def add_moveto_pdp(self, lineno, obj, surface, loc_name, req_side=None, pose_name=None):
    self._add_action(MovetoPDPAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, obj, surface, loc_name, req_side, pose_name))

  def add_pickup(self, lineno, obj, manip='rightarm', pose_name = ""):
    self._add_action(PickupAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, obj, manip, pose_name))

  def add_pickup_tray(self, lineno, tray, pose_name):
    self._add_action(PickupTrayAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, tray, pose_name))

  def add_putdown(self, lineno, obj, location_str, manip='rightarm', pose_name = ""):
    self._add_action(PutdownAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, obj, location_str, manip, pose_name))

  def add_putdown_tray(self, lineno, tray, table, pose_name):
    self._add_action(PutdownTrayAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, tray, table, pose_name))

  def add_handoff(self, lineno, arm_with_object, obj_name):
    self._add_action(handoff.HandoffAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, arm_with_object, obj_name))

  def add_open_drawer(self, lineno, drawer, manip, pose_name, open=True):
    self._add_action(drawer_action.DrawerAction(
      self.robot, self.pr2, self.unmovable_objects, lineno, drawer, manip, pose_name, open=open))
