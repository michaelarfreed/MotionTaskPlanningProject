import openravepy
import settings
from rapprentice import PR2
from trajevent import *
from motion_planner_wrapper import MotionPlannerWrapper
from base_pose_generator2 import BasePoseGenerator2
from base_around_pose_generator import BaseAroundPoseGenerator
from surface_location_generator import SurfaceLocationGenerator
if settings.DOMAIN == settings.CAN_DOMAIN:
  from pickup_pose_generator import PickupPoseGenerator
elif settings.DOMAIN in {settings.DINNER_DOMAIN, settings.DRAWER_DOMAIN}:
  from lip_pickup_pose_generator import LipPickupPoseGenerator as PickupPoseGenerator
else:
  raise Exception("Bad domain in settings!")
from tray_pickup_pose_generator import TrayPickupPoseGenerator
from putdown_pose_generator import PutdownPoseGenerator
from tray_putdown_pose_generator import TrayPutdownPoseGenerator
from collision_checker import CollisionChecker
from env_manager import EnvManager
import utils
import tray_world
import pdb
import IPython
import time

class ActionError(Exception):
  def __init__(self, problem, lineno, robot):
    self.problem = problem
    self.line_number = lineno
    self.robot = robot

class InstantiationExhaustedException(Exception):
  pass

class HLAction(object):
  last_target_cache = {}

  def __init__(self, robot, pr2, unmovable_objects, lineno):
    self.robot = robot
    self.env = robot.GetEnv()
    self.pr2 = pr2
    self.unmovable_objects = unmovable_objects
    self.motion_planner_wrapper = MotionPlannerWrapper(self.env, self.unmovable_objects)
    self.lineno = lineno
    self.collision_checker = CollisionChecker(self.env)

    self.target_generator = None
    self.last_trajevents = None
    self.next_target = None
    self.untried_target_poses = []
    self.untried_target_poses_iter = None
    self.unreachable = True

    self.cache_gen = self.get_cached_target()


  def get_name(self):
    return "name not set"


  def print_grab_details(self):
    lmanip = self.robot.GetManipulator('leftarm')
    rmanip = self.robot.GetManipulator('rightarm')
    # if self.get_name() == "movetogp object2 leftarm":
    #   IPython.embed()
    for obj in self.robot.GetGrabbed():
      print obj.GetName()+" grabbed by -- ",
      if lmanip.IsGrabbing(obj):
        print lmanip.GetName()
      if rmanip.IsGrabbing(obj):
        print rmanip.GetName()


  def reset(self):
    self.target_generator = None
    self.last_trajevents = None
    self.next_target = None
    self.untried_target_poses = []
    self.untried_target_poses_iter = None
    self.cache_gen = self.get_cached_target()
    self.unreachable = True

  def set_cache_target(self, target):
    HLAction.last_target_cache[self.last_target_cache_key] = target

  def get_cached_target(self):
    if hasattr(self, "base_pose_generator"):  # yield current pose first
      if type(self) == MovetoGPAction:
        yield self.base_pose_generator.generate_curr_pose(self.obj)
      elif type(self) == MovetoPDPAction:
        yield self.base_pose_generator.generate_curr_pose(self.surface)
    if self.last_target_cache_key is None:
      raise StopIteration
    if self.last_target_cache_key in HLAction.last_target_cache.keys():
      yield HLAction.last_target_cache[self.last_target_cache_key]

  def prep_next(self, error_free_only=True):
    next_target = None
    if self.target_generator is None:
      self._init_data_generator()
      try:
        next_target = next(self.cache_gen)
      except StopIteration:
        next_target = None

    if next_target is None:
      next_target = self.target_generator.next()

    self.next_target = next_target
    self.last_trajevents = None

  def find_and_execute_next_fastevents(self):
    fastevents = None
    while fastevents is None:
      self.prep_next()
      target_poses = self.next_target
      self.set_cache_target(self.next_target)
      fastevents = self.get_trajevents(
        target_poses, self.motion_planner_wrapper,
        error_free_only=True, fast=True)
    for fastevent in fastevents:
      fastevent.execute()
      # raw_input("watch!")

  def calc_trajevents(self):
    target_poses = self.next_target
    self.set_cache_target(self.next_target)
    trajevents = self.get_trajevents(
      target_poses, self.motion_planner_wrapper, error_free_only=True)
    if trajevents is None:
      raise StopIteration
    else:
      self.last_trajevents = trajevents

  def find_next_trajevents(self):
    trajevents = None
    while trajevents is None:
      try:
        self.prep_next(error_free_only=False)
        target_poses = self.next_target
      except StopIteration:
          break
      self.set_cache_target(self.next_target)
      trajevents = self.get_trajevents(
        target_poses, self.motion_planner_wrapper, error_free_only=True)
      if trajevents is None:
        self.untried_target_poses.append(target_poses)
      else:
        self.last_trajevents = trajevents
        self.unreachable = False

    # if type(self.untried_target_poses) == list:
    #   untried_target_poses_iter = iter(self.untried_target_poses)
    if self.untried_target_poses_iter is None:
      self.untried_target_poses_iter = iter(self.untried_target_poses)


    print "\n\nNo error free MP found. Allowing collisions.\n\n"
    while trajevents is None:
      try:
        target_poses = self.untried_target_poses_iter.next()
      except StopIteration, e:
        if self.unreachable and self.__class__ == PickupAction:
          self.raise_unreachable_error()
        else:
          raise e
      self.set_cache_target(target_poses)
      trajevents = self.get_trajevents(
        target_poses, self.motion_planner_wrapper, error_free_only=False)
      if trajevents is None:
        continue
      else:
        self.last_trajevents = trajevents
        self.unreachable = False

  def execute_trajevents(self, sim_only=True, use_pr2=False):
    if use_pr2:
      sim_only = False

    for trajevent in self.last_trajevents:
      trajevent.execute(sim_only=sim_only, use_pr2=use_pr2)

  def replan(self):
    print "Replanning..."
    target_poses = self.next_target
    trajevents = self.get_trajevents(
      target_poses, self.motion_planner_wrapper,
      error_free_only=True, replan=True)
    if trajevents is None:
      print "Replan Error!"
      self.replan()  # TODO: something funny is going on; some replans work on second call
      # IPython.embed()  # for debugging
      # raise Exception("Replan error!")
    else:
      self.last_trajevents = trajevents

  def raise_collisions_error(self, collisions, errtype="grasp", location=None):
    if errtype == "grasp":
      e = ActionError("Object in collision", self.lineno, self.robot)
    elif errtype == "putdown":
      e = ActionError("Object obstructing putdown", self.lineno, self.robot)
      e.location = location
    e.object_to_grasp = self.obj
    e.manip = self.manip
    e.pose_name = self.pose_name

    # return only movable collisions unless there are unmovable obstructions
    # a constraint based motion planner may be able to find a solution not in collisions with
    # unmovables once movable obstructions are removed
    movable_collisions = filter(lambda obj: obj not in self.unmovable_objects, collisions)
    if len(movable_collisions) > 0:
      e.collisions = [obj.GetName() for obj in movable_collisions]
    else:
      e.collisions = [obj.GetName() for obj in collisions]
    raise e

  def raise_unreachable_error(self):
    if self.pose_name == "" or self.pose_name is None:
      print "Unexpected error: no pose_name"
      pdb.set_trace()
    e = ActionError("Object unreachable", self.lineno, self.robot)
    e.object_to_grasp = self.obj
    e.manip = self.manip
    e.pose_name = self.pose_name
    raise e

  def raise_unstackable_error(self):
    e = ActionError("Incompatible objects", self.lineno, self.robot)
    e.object_to_grasp = self.obj  # this doesn't make sense, but it's the orig implementation...
    e.stacktop = self.env.GetKinBody(EnvManager.tray_stack[-1])
    raise e

  def check_for_wrongside_error(self, fast):
    RATIO_X_Y_THRESHOLD = 1.5

    assert hasattr(self, "req_side"), "Wrong side error not applicable to this action!"
    if fast or self.req_side is None:
      return
    if hasattr(self, "surface"):
      # for MovetoPDPAction, we consider location of surface instead of object currently held
      obj_to_use = self.surface
    else:
      obj_to_use = self.obj

    robot_t = self.robot.GetTransform()
    obj_t = np.linalg.inv(robot_t).dot(obj_to_use.GetTransform())
    obj_x, obj_y = obj_t[0, 3], obj_t[1, 3]
    if self.pose_name.startswith("l"):
      other_pose = "r" + self.pose_name[1:]
    elif self.pose_name.startswith("r"):
      other_pose = "l" + self.pose_name[1:]
    else:
      raise TypeError("Invalid pose string: %s" % self.pose_name)

    if abs(obj_x / obj_y) > RATIO_X_Y_THRESHOLD:
      if self.req_side in self.manip:
        return  # no wrongside error if object close to robot

    if (self.req_side == "left" and obj_y < 0):
      msg = "(onright %s)\n(not (onleft %s))" % (self.pose_name, self.pose_name)
      msg += "\n(onright %s)\n(not (onleft %s))" % (other_pose, other_pose)
      e = ActionError(msg, self.lineno, self.robot)
      e.object_to_grasp = obj_to_use
      raise e
    if (self.req_side == "right" and obj_y > 0):
      msg = "(onleft %s)\n(not (onright %s))" % (self.pose_name, self.pose_name)
      msg += "\n(onleft %s)\n(not (onright %s))" % (other_pose, other_pose)
      e = ActionError(msg, self.lineno, self.robot)
      e.object_to_grasp = obj_to_use
      raise e


class MovetoGPAction(HLAction):
  def __init__(self, robot, pr2, unmovable_objects, lineno, obj, manip, req_side=None, pose_name=None):
    super(MovetoGPAction, self).__init__(robot, pr2, unmovable_objects, lineno)
    self.obj = obj
    self.manip = manip
    self.base_pose_generator = BasePoseGenerator2(self.env, self.unmovable_objects)
    self.pickup_pose_generator = PickupPoseGenerator(self.env, self.unmovable_objects)
    self.last_target_cache_key = "MOVETOGP:{}_{}".format(
      self.obj.GetName(), self.manip)
    self.req_side = req_side
    self.pose_name = pose_name

  def get_name(self):
    return " ".join(["movetogp", self.obj.GetName(), self.manip])

  def _init_data_generator(self):
    self.target_generator = self.base_pose_generator.generate_gp_poses(self.obj)

  def get_trajevents(
    self, target_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    self.check_for_wrongside_error(fast)
    
    base_pose, _ = target_poses
    base_pose = self.base_pose_generator.get_world_frame_pose(
      base_pose, self.obj.GetTransform())

    # check for collisions and reachability
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      self.robot.SetTransform(base_pose)
      collisions = self.collision_checker.get_collisions()
      if (error_free_only and collisions != set()) or\
         (collisions.intersection(self.unmovable_objects) != set()):
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      # manip_to_use = self.robot.GetManipulator(self.manip)
      # utils.remove_movable_objects(
      #   self.env, self.unmovable_objects)

      # for pre_grasp_pose, grasp_pose, lift_pose in\
      #         self.pickup_pose_generator.generate_poses(self.obj):
      #   pre_grasp_pose = PickupPoseGenerator.get_world_frame_pose(
      #     pre_grasp_pose, self.obj.GetTransform())
      #   grasp_pose = PickupPoseGenerator.get_world_frame_pose(
      #     grasp_pose, self.obj.GetTransform())
      #   lift_pose = PickupPoseGenerator.get_world_frame_pose(
      #     lift_pose, self.obj.GetTransform())

      #   pre_grasp_ik = manip_to_use.FindIKSolution(
      #     pre_grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      #   if pre_grasp_ik is None:
      #     continue

      #   grasp_ik = manip_to_use.FindIKSolution(
      #     grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      #   if grasp_ik is None:
      #     continue

      #   lift_ik = manip_to_use.FindIKSolution(
      #     lift_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      #   if lift_ik is None:
      #     continue

      #   break
      # else:
      #   EnvManager.restore_openrave_state(self.env, saved_env)
      #   # return None
      EnvManager.restore_openrave_state(self.env, saved_env)

    # don't motion plan if start and end are same
    if np.allclose(base_pose, self.robot.GetTransform()):
      return []

    if fast:
      return [FastBase(self.robot, self.pr2, base_pose)]
    else:
      with self.env:
        base_pose = openravepy.poseFromMatrix(base_pose).tolist()
        if replan and self.last_trajevents != []:
          init_data = self.last_trajevents[0].traj
        else:
          init_data = None
        traj, cost, collisions = motion_planner_wrapper.traj_from_base_pose(
          base_pose, collisionfree=True, init_data=init_data)
        if collisions != set():
          print collisions
          return None
        else:
          return [GenericTraj(self.robot, self.pr2, traj, speed_factor=0.25)]


class MovetoPDPAction(HLAction):
  def __init__(self, robot, pr2, unmovable_objects, lineno, obj, surface, loc_name,
               req_side=None, pose_name=None):
    super(MovetoPDPAction, self).__init__(robot, pr2, unmovable_objects, lineno)
    self.obj = obj
    self.obj_name = self.obj.GetName() if self.obj is not None else 'None'
    self.surface = surface
    self.loc_name = loc_name if loc_name is not None else 'None'
    self.base_pose_generator = BaseAroundPoseGenerator(self.env)
    self.req_side = req_side
    self.pose_name = pose_name

  def get_name(self):
    return " ".join(["movetopdp", self.obj_name, self.surface.GetName(), self.loc_name])

  def _init_data_generator(self):
    self.last_target_cache_key = "MOVETOPDP:{}_{}_{}".format(
      self.surface.GetName(), self.obj_name, self.loc_name)
    self.target_generator = self.base_pose_generator.generate_poses(self.surface)

  def get_trajevents(
    self, target_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    self.check_for_wrongside_error(fast)

    base_pose = target_poses
    base_pose = self.base_pose_generator.get_world_frame_pose(
      base_pose, self.surface.GetTransform())

    # check for collisions
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      self.robot.SetTransform(base_pose)
      collisions = self.collision_checker.get_collisions()
      if (error_free_only and collisions != set()) or\
         (collisions.intersection(self.unmovable_objects) != set()):
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None
      EnvManager.restore_openrave_state(self.env, saved_env)

    # don't motion plan if start and end are same
    if np.allclose(base_pose, self.robot.GetTransform()):
      return []

    if fast:
      return [FastBase(self.robot, self.pr2, base_pose)]
    else:
      with self.env:
        base_pose = openravepy.poseFromMatrix(base_pose).tolist()
        if replan:
          init_data = self.last_trajevents[0].traj
        else:
          init_data = None
        traj, cost, collisions = motion_planner_wrapper.traj_from_base_pose(
          base_pose, collisionfree=True, init_data=init_data)
        if collisions != set():
          print collisions
          return None
        else:
          return [GenericTraj(self.robot, self.pr2, traj, speed_factor=0.25)]


class PickupAction(HLAction):
  def __init__(self, robot, pr2, unmovable_objects, lineno, obj, manip, pose_name):
    super(PickupAction, self).__init__(robot, pr2, unmovable_objects, lineno)
    self.obj = obj
    self.manip = manip
    self.pickup_pose_generator = PickupPoseGenerator(self.env, self.unmovable_objects)
    self.last_target_cache_key = "PICKUP:{}_{}".format(
      self.obj.GetName(), self.manip)
    self.pose_name = pose_name

  def get_name(self):
    return " ".join(["grasp", self.obj.GetName(), self.manip])

  def _init_data_generator(self):
    self.target_generator = self.pickup_pose_generator.generate_poses(self.obj)

  def get_trajevents(
    self, target_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    (pre_grasp_pose, grasp_pose, lift_pose) = target_poses

    pre_grasp_pose = PickupPoseGenerator.get_world_frame_pose(
      pre_grasp_pose, self.obj.GetTransform())
    grasp_pose = PickupPoseGenerator.get_world_frame_pose(
      grasp_pose, self.obj.GetTransform())
    lift_pose = PickupPoseGenerator.get_world_frame_pose(
      lift_pose, self.obj.GetTransform())

    # finding IKs
    manip_to_use = self.robot.GetManipulator(self.manip)
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      utils.open_gripper(self.robot, self.manip)

      pre_grasp_ik = manip_to_use.FindIKSolution(
        pre_grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      if pre_grasp_ik is None and error_free_only:
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      self.env.Remove(self.obj)
      grasp_ik = manip_to_use.FindIKSolution(
        grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      if grasp_ik is None and error_free_only:
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      lift_ik = manip_to_use.FindIKSolution(
        lift_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      if lift_ik is None and error_free_only:
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      utils.remove_movable_objects(
        self.env, self.unmovable_objects)

      if pre_grasp_ik is None:
        pre_grasp_ik = manip_to_use.FindIKSolution(
          pre_grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if pre_grasp_ik is None:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      if grasp_ik is None:
        grasp_ik = manip_to_use.FindIKSolution(
          grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if grasp_ik is None:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      if lift_ik is None:
        lift_ik = manip_to_use.FindIKSolution(
          lift_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if lift_ik is None:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      EnvManager.restore_openrave_state(self.env, saved_env)

    # side posture joints
    left_joints = utils.constrain_within_joint_limits(
      self.robot, PR2.Arm.L_POSTURES['side2'], arm='leftarm')
    right_joints = utils.constrain_within_joint_limits(
      self.robot, PR2.mirror_arm_joints(left_joints).tolist(), arm='rightarm')

    if fast:
      return [OpenGripperEvent(self.robot, self.pr2, self.manip),
              FastArm(self.robot, self.pr2, pre_grasp_ik, self.manip),
              FastArm(self.robot, self.pr2, grasp_ik, self.manip),
              GrabEvent(self.robot, self.pr2, self.obj, self.manip),
              FastArm(self.robot, self.pr2, lift_ik, self.manip),
              FastArm(self.robot, self.pr2, left_joints, 'leftarm'),
              FastArm(self.robot, self.pr2, right_joints, 'rightarm')]
    else:
      with self.env:
        saved_env = EnvManager.save_openrave_state(self.env)

        self.robot.SetActiveManipulator(self.manip)
        utils.open_gripper(self.robot, self.manip)

        # finding trajs
        pre_grasp_pose = openravepy.poseFromMatrix(pre_grasp_pose).tolist()
        grasp_pose = openravepy.poseFromMatrix(grasp_pose).tolist()
        lift_pose = openravepy.poseFromMatrix(lift_pose).tolist()

        if replan:
          init_data1 = self.last_trajevents[1].traj
        else:
          init_data1 = None
        traj1, cost1, col1 = motion_planner_wrapper.traj_from_pose(
          {self.manip: pre_grasp_pose}, collisionfree=error_free_only,
          init_dofs=utils.extend_dofs(self.robot, pre_grasp_ik, self.manip),
          init_data=init_data1)
        if error_free_only and col1 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col1
          return None

        utils.update_robot(self.robot, traj1[-1])
        self.env.Remove(self.obj)
        if replan:
          init_data2 = self.last_trajevents[2].traj
        else:
          init_data2 = None
        traj2, cost2, col2 = motion_planner_wrapper.traj_from_pose(
          {self.manip: grasp_pose}, collisionfree=error_free_only,
          init_dofs=utils.extend_dofs(self.robot, grasp_ik, self.manip),
          init_data=init_data2, maintain_linear=True)
        self.env.AddKinBody(self.obj)
        if error_free_only and col2 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col2
          return None

        utils.update_robot(self.robot, traj2[-1])
        utils.grab_helper(self.robot, self.obj, self.manip)
        if replan:
          init_data3 = self.last_trajevents[4].traj
        else:
          init_data3 = None
        traj3, cost3, col3 = motion_planner_wrapper.traj_from_pose(
          {self.manip: lift_pose}, collisionfree=error_free_only,
          init_dofs=utils.extend_dofs(self.robot, lift_ik, self.manip),
          init_data=init_data3, include_obj_world_col=False)
        if error_free_only and col3 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col3
          return None

        # side posture
        utils.update_robot(self.robot, traj3[-1])
        if replan:
          init_data4 = self.last_trajevents[5].traj
        else:
          init_data4 = None
        traj4, cost4, col4 = motion_planner_wrapper.traj_from_dofs(
          right_joints + left_joints, collisionfree=error_free_only,
          init_data=init_data4)

        EnvManager.restore_openrave_state(self.env, saved_env)
        collisions = col1.union(col2).union(col3).union(col4)
        if collisions:
          if error_free_only:
            print collisions
            return None
          else:
            self.raise_collisions_error(collisions)

        return [OpenGripperEvent(self.robot, self.pr2, self.manip),
                GenericTraj(self.robot, self.pr2, traj1),
                GenericTraj(self.robot, self.pr2, traj2, speed_factor=0.1),
                GrabEvent(self.robot, self.pr2, self.obj, self.manip),
                GenericTraj(self.robot, self.pr2, traj3, speed_factor=0.1),
                GenericTraj(self.robot, self.pr2, traj4)]


class PickupTrayAction(HLAction):
  def __init__(self, robot, pr2, unmovable_objects, lineno, tray, pose_name):
    super(PickupTrayAction, self).__init__(robot, pr2, unmovable_objects, lineno)
    self.tray = tray
    self.pickup_pose_generator = TrayPickupPoseGenerator(self.env, self.unmovable_objects)
    self.last_target_cache_key = "PICKUPTRAY:{}".format(self.tray.GetName())
    self.pose_name = pose_name

  def get_name(self):
    return "picktray"

  def _init_data_generator(self):
    self.target_generator = self.pickup_pose_generator.generate_poses(self.tray)

  def get_trajevents(
    self, target_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    target_poses_world = []
    tray_t = self.tray.GetTransform()
    for i, pose in enumerate(target_poses):
      target_poses_world.append(PickupPoseGenerator.get_world_frame_pose(
        pose, tray_t))

    (r_pre_grasp_pose, r_grasp_pose, r_lift_pose,
     l_pre_grasp_pose, l_grasp_pose, l_lift_pose) = target_poses_world
    poses_by_arm = {'rightarm': (r_pre_grasp_pose, r_grasp_pose, r_lift_pose),
                    'leftarm': (l_pre_grasp_pose, l_grasp_pose, l_lift_pose)}
    # finding IKs
    iks_by_arm = {}
    with self.env:
      for manip, (pre_grasp_pose, grasp_pose, lift_pose) in poses_by_arm.items():
        manip_to_use = self.robot.GetManipulator(manip)

        saved_env = EnvManager.save_openrave_state(self.env)
        utils.open_gripper(self.robot, manip)

        pre_grasp_ik = manip_to_use.FindIKSolution(
          pre_grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if pre_grasp_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

        self.env.Remove(self.tray)
        grasp_ik = manip_to_use.FindIKSolution(
          grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if grasp_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

        lift_ik = manip_to_use.FindIKSolution(
          lift_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if lift_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

        utils.remove_movable_objects(
          self.env, self.unmovable_objects)

        if pre_grasp_ik is None:
          pre_grasp_ik = manip_to_use.FindIKSolution(
            pre_grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
          if pre_grasp_ik is None:
            EnvManager.restore_openrave_state(self.env, saved_env)
            return None

        if grasp_ik is None:
          grasp_ik = manip_to_use.FindIKSolution(
            grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
          if grasp_ik is None:
            EnvManager.restore_openrave_state(self.env, saved_env)
            return None

        if lift_ik is None:
          lift_ik = manip_to_use.FindIKSolution(
            lift_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
          if lift_ik is None:
            EnvManager.restore_openrave_state(self.env, saved_env)
            return None

        EnvManager.restore_openrave_state(self.env, saved_env)
        iks_by_arm[manip] = (pre_grasp_ik, grasp_ik, lift_ik)

    r_pre_grasp_ik, r_grasp_ik, r_lift_ik = iks_by_arm['rightarm']
    l_pre_grasp_ik, l_grasp_ik, l_lift_ik = iks_by_arm['leftarm']

    if fast:
      return [OpenGripperEvent(self.robot, self.pr2, 'rightarm'),
              OpenGripperEvent(self.robot, self.pr2, 'leftarm'),
              FastArm(self.robot, self.pr2, r_pre_grasp_ik, 'rightarm'),
              FastArm(self.robot, self.pr2, l_pre_grasp_ik, 'leftarm'),
              FastArm(self.robot, self.pr2, r_grasp_ik, 'rightarm'),
              FastArm(self.robot, self.pr2, l_grasp_ik, 'leftarm'),
              GrabEvent(self.robot, self.pr2, self.tray, 'rightarm'),
              GrabEvent(self.robot, self.pr2, self.tray, 'leftarm'),
              FastArm(self.robot, self.pr2, r_lift_ik, 'rightarm'),
              FastArm(self.robot, self.pr2, l_lift_ik, 'leftarm')]
    else:
      with self.env:
        saved_env = EnvManager.save_openrave_state(self.env)

        # self.robot.SetActiveManipulator(self.manip)
        utils.open_gripper(self.robot, 'rightarm')
        utils.open_gripper(self.robot, 'leftarm')

        # finding trajs
        r_pre_grasp_pose = openravepy.poseFromMatrix(r_pre_grasp_pose).tolist()
        r_grasp_pose = openravepy.poseFromMatrix(r_grasp_pose).tolist()
        r_lift_pose = openravepy.poseFromMatrix(r_lift_pose).tolist()

        l_pre_grasp_pose = openravepy.poseFromMatrix(l_pre_grasp_pose).tolist()
        l_grasp_pose = openravepy.poseFromMatrix(l_grasp_pose).tolist()
        l_lift_pose = openravepy.poseFromMatrix(l_lift_pose).tolist()

        if replan:
          init_data1 = self.last_trajevents[2].traj
        else:
          init_data1 = None
        traj1, cost1, col1 = motion_planner_wrapper.traj_from_pose(
          {'rightarm': r_pre_grasp_pose, 'leftarm': l_pre_grasp_pose},
          collisionfree=error_free_only,
          init_dofs=r_pre_grasp_ik.tolist() + l_pre_grasp_ik.tolist(),
          init_data=init_data1)
        if error_free_only and col1 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col1
          return None

        utils.update_robot(self.robot, traj1[-1])
        self.env.Remove(self.tray)
        if replan:
          init_data2 = self.last_trajevents[3].traj
        else:
          init_data2 = None
        traj2, cost2, col2 = motion_planner_wrapper.traj_from_pose(
          {'rightarm': r_grasp_pose, 'leftarm': l_grasp_pose},
          collisionfree=error_free_only,
          init_dofs=r_grasp_ik.tolist() + l_grasp_ik.tolist(),
          init_data=init_data2, maintain_linear=True)
        self.env.AddKinBody(self.tray)
        if error_free_only and col2 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col2
          return None

        # don't check for collisions during lift
        utils.update_robot(self.robot, traj2[-1])
        utils.grab_helper(self.robot, self.tray, 'rightarm')
        utils.grab_helper(self.robot, self.tray, 'leftarm')
        if replan:
          init_data3 = self.last_trajevents[6].traj
        else:
          init_data3 = None
        traj3, cost3, _ = motion_planner_wrapper.traj_from_pose(
          {'rightarm': r_lift_pose, 'leftarm': l_lift_pose},
          collisionfree=error_free_only,
          init_dofs=r_lift_ik.tolist() + l_lift_ik.tolist(),
          maintain_rel=True, maintain_up=True, maintain_linear=True,
          init_data=init_data3, include_obj_world_col=False)

        EnvManager.restore_openrave_state(self.env, saved_env)
        collisions = col1.union(col2)
        if collisions:
          if error_free_only:
            print collisions
            return None
          else:
            self.raise_collisions_error(collisions)

        return [OpenGripperEvent(self.robot, self.pr2, 'rightarm'),
                OpenGripperEvent(self.robot, self.pr2, 'leftarm'),
                GenericTraj(self.robot, self.pr2, traj1),
                GenericTraj(self.robot, self.pr2, traj2, speed_factor=0.1),
                GrabEvent(self.robot, self.pr2, self.tray, 'rightarm'),
                GrabEvent(self.robot, self.pr2, self.tray, 'leftarm'),
                GenericTraj(self.robot, self.pr2, traj3, speed_factor=0.1)]


class PutdownAction(HLAction):
  def __init__(self, robot, pr2, unmovable_objects, lineno, obj, location_str, manip, pose_name):
    super(PutdownAction, self).__init__(robot, pr2, unmovable_objects, lineno)
    self.obj = obj
    self.location_str = location_str
    if settings.REPORT_PUTDOWN_OBSTRUCTIONS:
      table_str, loc_name = self.location_str.split('_')
      self.table = self.env.GetKinBody(table_str)
      self.loc_name = loc_name
    else:
      self.table = self.env.GetKinBody(self.location_str)
      self.loc_name = 'None'
    self.manip = manip
    self.surface_location_generator = SurfaceLocationGenerator(self.env, self.unmovable_objects)
    self.putdown_pose_generator = PutdownPoseGenerator(self.env, self.unmovable_objects)
    self.pose_name = pose_name

  def get_name(self):
    return " ".join(["putdown", self.obj.GetName(), self.table.GetName(), self.loc_name, self.manip])

  def _init_data_generator(self):
    self.last_target_cache_key = "PUTDOWN:{}_{}_{}".format(
      self.obj.GetName(), self.location_str, self.manip)
    # self.target_generator = self.putdown_pose_generator.generate_poses(
    #   self.obj, self.manip)
    self.target_generator = self.putdown_pose_generator.generate_pdp_for_surface(self.table, self.obj, self.manip)

  def get_trajevents(
    self, target_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    (loc_t, (pre_pose, putdown_pose, retreat_pose)) = target_poses

    if self.table.GetName() == 'tray':
      tray_stack = EnvManager.tray_stack
      if not len(tray_stack) == 0 and\
         not tray_world.can_stack(tray_stack[-1], self.obj, self.env):
        if not fast:
          self.raise_unstackable_error()

    pre_pose = PutdownPoseGenerator.get_world_frame_pose(
      pre_pose, loc_t)
    putdown_pose = PutdownPoseGenerator.get_world_frame_pose(
      putdown_pose, loc_t)
    retreat_pose = PutdownPoseGenerator.get_world_frame_pose(
      retreat_pose, loc_t)


    # finding IKs
    # TODO[SS] long monolithic etc. break into IK and traj methods
    manip_to_use = self.robot.GetManipulator(self.manip)
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      for obj in self.robot.GetGrabbed():
        self.robot.Release(obj)
        self.env.Remove(obj)

      try:
        x=utils.plot_transform(self.env, pre_pose, s=0.1)
        print 0

        pre_ik = manip_to_use.FindIKSolution(
          pre_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if pre_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None
        print 1
        utils.open_gripper(self.robot, self.manip)
        putdown_ik = manip_to_use.FindIKSolution(
          putdown_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if putdown_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None
        print 2
        retreat_ik = manip_to_use.FindIKSolution(
          retreat_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if retreat_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None
        print 3
      except Exception as e:
        # TODO: sometimes an openrave pointer exception gets thrown
        print "caught exception"
        pdb.set_trace()
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      EnvManager.restore_openrave_state(self.env, saved_env)

      utils.remove_movable_objects(
        self.env, self.unmovable_objects)

      if pre_ik is None:
        pre_ik = manip_to_use.FindIKSolution(
          pre_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if pre_ik is None:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      if putdown_ik is None:
        utils.open_gripper(self.robot, self.manip)
        putdown_ik = manip_to_use.FindIKSolution(
          putdown_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if putdown_ik is None:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      if retreat_ik is None:
        utils.open_gripper(self.robot, self.manip)
        retreat_ik = manip_to_use.FindIKSolution(
          retreat_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if retreat_ik is None:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      EnvManager.restore_openrave_state(self.env, saved_env)

    # side posture joints
    left_joints = utils.constrain_within_joint_limits(
      self.robot, PR2.Arm.L_POSTURES['side2'], arm='leftarm')
    right_joints = utils.constrain_within_joint_limits(
      self.robot, PR2.mirror_arm_joints(left_joints).tolist(), arm='rightarm')

    if fast:
      return [FastArm(self.robot, self.pr2, pre_ik, self.manip),
              FastArm(self.robot, self.pr2, putdown_ik, self.manip),
              ReleaseEvent(self.robot, self.pr2, self.obj, self.table, self.manip),
              FastArm(self.robot, self.pr2, retreat_ik, self.manip),
              FastArm(self.robot, self.pr2, left_joints, 'leftarm'),
              FastArm(self.robot, self.pr2, right_joints, 'rightarm')]
    else:
      with self.env:
        saved_env = EnvManager.save_openrave_state(self.env)

        self.robot.SetActiveManipulator(self.manip)

        # finding trajs
        pre_pose = openravepy.poseFromMatrix(pre_pose).tolist()
        putdown_pose = openravepy.poseFromMatrix(putdown_pose).tolist()
        retreat_pose = openravepy.poseFromMatrix(retreat_pose).tolist()

        if replan:
          init_data0 = self.last_trajevents[0].traj
        else:
          init_data0 = None
        traj0, cost0, col0 = motion_planner_wrapper.traj_from_pose(
          {self.manip: pre_pose},
          init_dofs=utils.extend_dofs(self.robot, pre_ik, self.manip),
          init_data=init_data0)
        if error_free_only and col0 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col0
          return None

        utils.update_robot(self.robot, traj0[-1])
        if replan:
          init_data1 = self.last_trajevents[1].traj
        else:
          init_data1 = None
        traj1, cost1, col1 = motion_planner_wrapper.traj_from_pose(
          {self.manip: putdown_pose},
          init_dofs=utils.extend_dofs(self.robot, putdown_ik, self.manip),
          init_data=init_data1, maintain_linear=True, include_obj_world_col=False)
        if error_free_only and col1 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col1
          return None

        utils.update_robot(self.robot, traj1[-1])
        utils.open_gripper(self.robot, self.manip)
        self.env.Remove(self.obj)
        if replan:
          init_data2 = self.last_trajevents[3].traj
        else:
          init_data2 = None
        traj2, cost2, col2 = motion_planner_wrapper.traj_from_pose(
          {self.manip: retreat_pose},
          init_dofs=utils.extend_dofs(self.robot, retreat_ik, self.manip),
          init_data=init_data2, maintain_linear=True)
        self.env.AddKinBody(self.obj)
        if error_free_only and col2 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col2
          return None

        # side posture
        utils.update_robot(self.robot, traj2[-1])
        utils.release_helper(self.robot, self.obj, self.manip)
        if replan:
          init_data3 = self.last_trajevents[4].traj
        else:
          init_data3 = None
        traj3, cost3, col3 = motion_planner_wrapper.traj_from_dofs(
          right_joints + left_joints, collisionfree=error_free_only,
          init_data=init_data3)

        EnvManager.restore_openrave_state(self.env, saved_env)

        collisions = col0.union(col1).union(col2).union(col3)
        if collisions:
          print "putdown collisions: " + repr(collisions)
          if error_free_only:
            return None
          else:
            if settings.REPORT_PUTDOWN_OBSTRUCTIONS:
              self.raise_collisions_error(
                collisions, errtype="putdown", location=self.location_str)
            else:
              return None

        return [GenericTraj(self.robot, self.pr2, traj0, speed_factor=0.5),
                GenericTraj(self.robot, self.pr2, traj1, speed_factor=0.1),
                ReleaseEvent(self.robot, self.pr2, self.obj, self.table, self.manip),
                GenericTraj(self.robot, self.pr2, traj2),
                GenericTraj(self.robot, self.pr2, traj3)]


class PutdownTrayAction(HLAction):
  def __init__(self, robot, pr2, unmovable_objects, lineno, tray, table, pose_name):
    super(PutdownTrayAction, self).__init__(robot, pr2, unmovable_objects, lineno)
    self.tray = tray
    self.table = table
    self.surface_location_generator = SurfaceLocationGenerator(self.env, self.unmovable_objects)
    self.putdown_pose_generator = TrayPutdownPoseGenerator(self.env, self.unmovable_objects)
    self.pose_name = pose_name

  def get_name(self):
    return "putdowntray"

  def _init_data_generator(self):
    self.last_target_cache_key = "PUTDOWNTRAY:{}_{}".format(
      self.tray.GetName(), self.table.GetName())
    self.target_generator = self.putdown_pose_generator.generate_poses(self.tray)

  def get_trajevents(
    self, target_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    target_poses_world = []
    for i, pose in enumerate(target_poses):
      target_poses_world.append(PickupPoseGenerator.get_world_frame_pose(
        pose, self.table.GetTransform()))

    (r_pre_pose, r_putdown_pose, r_retreat_pose,
     l_pre_pose, l_putdown_pose, l_retreat_pose) = target_poses_world

    poses_by_arm = {'rightarm': (r_pre_pose, r_putdown_pose, r_retreat_pose),
                    'leftarm': (l_pre_pose, l_putdown_pose, l_retreat_pose)}

    # finding IKs
    iks_by_arm = {}
    with self.env:
      for manip, (pre_pose, putdown_pose, retreat_pose) in poses_by_arm.items():
        manip_to_use = self.robot.GetManipulator(manip)

        saved_env = EnvManager.save_openrave_state(self.env)
        utils.open_gripper(self.robot, manip)
        self.env.Remove(self.tray)

        pre_pose_ik = manip_to_use.FindIKSolution(
          pre_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if pre_pose_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

        putdown_ik = manip_to_use.FindIKSolution(
          putdown_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if putdown_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

        retreat_ik = manip_to_use.FindIKSolution(
          retreat_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if retreat_ik is None and error_free_only:
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

        utils.remove_movable_objects(
          self.env, self.unmovable_objects)

        if pre_pose_ik is None:
          pre_pose_ik = manip_to_use.FindIKSolution(
            pre_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
          if pre_pose_ik is None:
            EnvManager.restore_openrave_state(self.env, saved_env)
            return None

        if putdown_ik is None:
          putdown_ik = manip_to_use.FindIKSolution(
            putdown_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
          if putdown_ik is None:
            EnvManager.restore_openrave_state(self.env, saved_env)
            return None

        if retreat_ik is None:
          retreat_ik = manip_to_use.FindIKSolution(
            retreat_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
          if retreat_ik is None:
            EnvManager.restore_openrave_state(self.env, saved_env)
            return None

        EnvManager.restore_openrave_state(self.env, saved_env)
        iks_by_arm[manip] = (pre_pose_ik, putdown_ik, retreat_ik)

    r_pre_pose_ik, r_putdown_ik, r_retreat_ik = iks_by_arm['rightarm']
    l_pre_pose_ik, l_putdown_ik, l_retreat_ik = iks_by_arm['leftarm']

    if fast:
      return [FastArm(self.robot, self.pr2, l_pre_pose_ik, 'leftarm'),
              FastArm(self.robot, self.pr2, r_putdown_ik, 'rightarm'),
              FastArm(self.robot, self.pr2, l_putdown_ik, 'leftarm'),
              ReleaseEvent(self.robot, self.pr2, self.tray, self.table, 'rightarm'),
              ReleaseEvent(self.robot, self.pr2, self.tray, self.table, 'leftarm'),
              FastArm(self.robot, self.pr2, r_retreat_ik, 'rightarm'),
              FastArm(self.robot, self.pr2, l_retreat_ik, 'leftarm')]
    else:
      with self.env:
        saved_env = EnvManager.save_openrave_state(self.env)

        utils.open_gripper(self.robot, 'rightarm')
        utils.open_gripper(self.robot, 'leftarm')

        # finding trajs
        r_pre_pose = openravepy.poseFromMatrix(r_pre_pose).tolist()
        r_putdown_pose = openravepy.poseFromMatrix(r_putdown_pose).tolist()
        r_retreat_pose = openravepy.poseFromMatrix(r_retreat_pose).tolist()

        l_pre_pose = openravepy.poseFromMatrix(l_pre_pose).tolist()
        l_putdown_pose = openravepy.poseFromMatrix(l_putdown_pose).tolist()
        l_retreat_pose = openravepy.poseFromMatrix(l_retreat_pose).tolist()

        if replan:
          init_data1 = self.last_trajevents[0].traj
        else:
          init_data1 = None
        traj1, cost1, col1 = motion_planner_wrapper.traj_from_pose(
          {'rightarm': r_pre_pose, 'leftarm': l_pre_pose},
          collisionfree=error_free_only,
          init_dofs=r_pre_pose_ik.tolist() + l_pre_pose_ik.tolist(),
          maintain_rel=True, maintain_up=True, init_data=init_data1)
        if error_free_only and col1 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col1
          return None

        utils.update_robot(self.robot, traj1[-1])
        if replan:
          init_data2 = self.last_trajevents[1].traj
        else:
          init_data2 = None
        traj2, cost2, col2 = motion_planner_wrapper.traj_from_pose(
          {'rightarm': r_putdown_pose, 'leftarm': l_putdown_pose},
          collisionfree=error_free_only,
          init_dofs=r_putdown_ik.tolist() + l_putdown_ik.tolist(),
          maintain_rel=True, maintain_up=True, init_data=init_data2,
          maintain_linear=True, include_obj_world_col=False)
        if error_free_only and col2 != set():
          EnvManager.restore_openrave_state(self.env, saved_env)
          print col2
          return None

        # don't check for collisions during retreat
        utils.update_robot(self.robot, traj2[-1])
        self.env.Remove(self.tray)
        if replan:
          init_data3 = self.last_trajevents[4].traj
        else:
          init_data3 = None
        traj3, cost3, _ = motion_planner_wrapper.traj_from_pose(
          {'rightarm': r_retreat_pose, 'leftarm': l_retreat_pose},
          collisionfree=error_free_only,
          init_dofs=r_retreat_ik.tolist() + l_retreat_ik.tolist(),
          init_data=init_data3, maintain_linear=True)
        self.env.AddKinBody(self.tray)

        EnvManager.restore_openrave_state(self.env, saved_env)
        collisions = col1.union(col2)
        if collisions:
          if error_free_only:
            print collisions
            return None
          else:
            self.raise_collisions_error(collisions)

        return [GenericTraj(self.robot, self.pr2, traj1, speed_factor=0.1),
                GenericTraj(self.robot, self.pr2, traj2, speed_factor=0.1),
                ReleaseEvent(self.robot, self.pr2, self.tray, self.table, 'rightarm'),
                ReleaseEvent(self.robot, self.pr2, self.tray, self.table, 'leftarm'),
                GenericTraj(self.robot, self.pr2, traj3)]
