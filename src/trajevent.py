import utils
import numpy as np
import openravepy
import time
from motion_planner_wrapper import MotionPlannerWrapper
from rapprentice.pr2_trajectories import follow_body_traj
from settings import DISABLE_BASE
from env_manager import EnvManager

GRABBED_THRESH = 0.0


class TrajEvent(object):
  def __init__(self, robot, pr2):
    self.env = robot.GetEnv()
    self.robot = robot
    self.pr2 = pr2

  def _iterative_grasp(self, manip, open_amount=0.54, effort=80):
      if manip == 'rightarm':
        joint = 'r_gripper_joint'
        tool_frame = 'r_gripper_tool_frame'
      elif manip == 'leftarm':
        joint = 'l_gripper_joint'
        tool_frame = 'l_gripper_tool_frame'

      starting_tool_frame_t = self.robot.GetLink(tool_frame).GetTransform()
      starting_pose = openravepy.poseFromMatrix(starting_tool_frame_t).tolist()
      starting_pose[:4] = openravepy.quatMultiply(
        starting_pose[:4], openravepy.quatFromAxisAngle((0, np.pi/2, 0))).tolist()

      max_iters = 5
      while True:
        print("Closing {} gripper...").format(manip)
        if manip == 'rightarm':
          self.pr2.rgrip.close(max_effort=80)
        elif manip == 'leftarm':
          self.pr2.lgrip.close(max_effort=80)
        self.pr2.join_all()

        # check if object has been grabbed
        # if not, move in and try grab again
        time.sleep(1.0)
        self.pr2.update_rave()
        gripper_val = self.robot.GetDOFValues([self.robot.GetJoint(joint).GetDOFIndex()])[0]
        print("Gripper val: {}").format(gripper_val)

        if gripper_val > GRABBED_THRESH:
          print("Grab succeeded!")
          return

        max_iters -= 1
        if max_iters == 0:
          print "Hit max iterative grasp limit"
          return

        print("Grab failed. Trying again...")

        # prep for retry lower
        tool_frame_t = self.robot.GetLink(tool_frame).GetTransform()
        adjust_t = openravepy.matrixFromPose((1, 0, 0, 0, 0.01, 0, 0))
        new_target_t = tool_frame_t.dot(adjust_t)
        new_target_pose = openravepy.poseFromMatrix(new_target_t).tolist()
        new_target_pose[:4] = openravepy.quatMultiply(
          new_target_pose[:4], openravepy.quatFromAxisAngle((0, np.pi/2, 0))).tolist()

        # return to starting pose
        print "Returning to starting pose"
        saved_env = EnvManager.save_openrave_state(self.env)
        utils.remove_movable_objects(self.env, set())
        motion_planner_wrapper = MotionPlannerWrapper(self.env)
        traj0, _, _ = motion_planner_wrapper.traj_from_pose(
          {manip: starting_pose},
          collisionfree=False)
        EnvManager.restore_openrave_state(self.env, saved_env)
        generic_traj = GenericTraj(self.robot, self.pr2, traj0.tolist(), speed_factor=0.1)
        generic_traj.execute(sim_only=False)
        time.sleep(1.0)

        # open gripper
        print "Opening Gripper"
        if manip == 'rightarm':
          self.pr2.rgrip.open(scale_factor=open_amount / 0.54)
        elif manip == 'leftarm':
          self.pr2.lgrip.open(scale_factor=open_amount / 0.54)
        self.pr2.join_all()
        time.sleep(1.0)

        # retry lower
        print "Retrying with lower pose"
        saved_env = EnvManager.save_openrave_state(self.env)
        utils.remove_movable_objects(self.env, set())
        motion_planner_wrapper = MotionPlannerWrapper(self.env)
        traj, _, _ = motion_planner_wrapper.traj_from_pose(
          {manip: new_target_pose},
          collisionfree=False)
        EnvManager.restore_openrave_state(self.env, saved_env)
        generic_traj = GenericTraj(self.robot, self.pr2, traj.tolist(), speed_factor=0.1)
        generic_traj.execute(sim_only=False)


class GenericTraj(TrajEvent):
  offset_t = None

  def __init__(self, robot, pr2, traj, speed_factor=1.0):
    super(GenericTraj, self).__init__(robot, pr2)
    self.traj = traj
    self.speed_factor = speed_factor

  def execute(self, sim_only=True, use_pr2=False):
    if DISABLE_BASE and len(self.traj[0]) == 17:
      return

    if sim_only:
      animationtime = 0.0
    else:
      animationtime = 0.5
      starting_env = EnvManager.save_openrave_state(self.env)  # needed for syncing below
    utils.run_trajectory(self.robot, self.traj, animationtime=animationtime)

    if not sim_only and use_pr2:
      n_dofs = len(self.traj[0])
      print("Executing {} DOF trajectory...".format(n_dofs))
      with self.env:
        saved_env = EnvManager.save_openrave_state(self.env)
        # reset; needed to sync real and sim world
        EnvManager.restore_openrave_state(self.env, starting_env)

        traj_r = []
        traj_l = []
        traj_pos = []
        for joints in self.traj:
          traj_r.append(joints[:7])
          traj_l.append(joints[7:14])
          if n_dofs == 17:
            traj_pos.append(joints[14:17])
        bodypart_traj = {'rarm': np.array(traj_r),
                         'larm': np.array(traj_l)}
        if n_dofs == 17:
          # sync real and sim world
          robot_t = self.robot.GetTransform()
          if GenericTraj.offset_t is None:
            # Solving: pr2_t = offset_t * robot_t -> offset_t = pr2_t * inv(robot_t)
            pr2_t = utils.base_pose_to_mat(self.pr2.base.get_pose("/map"))
            GenericTraj.offset_t = pr2_t.dot(np.linalg.inv(robot_t))

          for i, p in enumerate(traj_pos):
            # Solving: world_t = offset_t * rave_world_t
            rave_world_t = utils.base_pose_to_mat(p)
            traj_pos[i] = utils.mat_to_base_pose(GenericTraj.offset_t.dot(rave_world_t))
          bodypart_traj['base'] = np.array(traj_pos)
        follow_body_traj(self.pr2, bodypart_traj, speed_factor=self.speed_factor)
        # reset
        EnvManager.restore_openrave_state(self.env, saved_env)
      print("Trajectory execution complete!")


class FastBase(TrajEvent):
  def __init__(self, robot, pr2, base_transform):
    super(FastBase, self).__init__(robot, pr2)
    self.base_transform = base_transform

  def execute(self, sim_only=True, use_pr2=False):
    if DISABLE_BASE:
      return

    self.robot.SetTransform(self.base_transform)


class FastArm(TrajEvent):
  def __init__(self, robot, pr2, dofs, manip):
    super(FastArm, self).__init__(robot, pr2)
    self.dofs = dofs
    self.manip = manip

  def execute(self, sim_only=True, use_pr2=False):
    self.robot.SetDOFValues(
      self.dofs, self.robot.GetManipulator(self.manip).GetArmIndices())


class OpenGripperEvent(TrajEvent):
  def __init__(self, robot, pr2, manip, value=0.54):
    super(OpenGripperEvent, self).__init__(robot, pr2)
    self.manip = manip
    self.value = value

  def execute(self, sim_only=True, use_pr2=False):
    utils.open_gripper(self.robot, self.manip, self.value)

    if not sim_only and use_pr2:
      print("Opening {} gripper...").format(self.manip)
      if self.manip == 'rightarm':
        self.pr2.rgrip.open(scale_factor=self.value / 0.54)
      elif self.manip == 'leftarm':
        self.pr2.lgrip.open(scale_factor=self.value / 0.54)
      self.pr2.join_all()
      print("Opening gripper complete!")


class GrabEvent(TrajEvent):
  def __init__(self, robot, pr2, obj, manip):
    super(GrabEvent, self).__init__(robot, pr2)
    self.obj = obj
    self.manip = manip

  def execute(self, sim_only=True, use_pr2=False):
    if self.obj.GetName() == 'tray':
      for obj in EnvManager.tray_stack:
        obj = self.env.GetKinBody(obj)
        utils.grab_helper(self.robot, obj, self.manip)
      utils.exclude_all_collisions(self.env, EnvManager.tray_stack + ['tray'])
    if self.obj.GetName() in EnvManager.tray_stack:
      if self.obj.GetName() == EnvManager.tray_stack[-1]:
        EnvManager.tray_stack.pop()
        EnvManager.tray_stack_height -= 0.035
      else:
        raise Exception("Trying to remove an object that isn't at the\
          top of the tray stack!")

    if not sim_only and use_pr2:
      print("Closing {} gripper...").format(self.manip)
      if self.manip == 'rightarm':
        self.pr2.rgrip.close(max_effort=80)
      elif self.manip == 'leftarm':
        self.pr2.lgrip.close(max_effort=80)
      self.pr2.join_all()

      # check if object has been grabbed
      # if not, move in and try grab again
      time.sleep(1.0)
      self.pr2.update_rave()

      if self.manip == 'rightarm':
        joint = 'r_gripper_joint'
        tool_frame = 'r_gripper_tool_frame'
      elif self.manip == 'leftarm':
        joint = 'l_gripper_joint'
        tool_frame = 'l_gripper_tool_frame'

      gripper_val = self.robot.GetDOFValues([self.robot.GetJoint(joint).GetDOFIndex()])[0]
      print("Gripper val: {}").format(gripper_val)
      if gripper_val <= GRABBED_THRESH:
        print("Grab failed. Trying again...")

        if self.manip == 'rightarm':
          self.pr2.rgrip.open()
        elif self.manip == 'leftarm':
          self.pr2.lgrip.open()

        tool_frame_t = self.robot.GetLink(tool_frame).GetTransform()
        adjust_t = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -0.01))
        new_target_t = adjust_t.dot(tool_frame_t)
        new_target_pose = openravepy.poseFromMatrix(new_target_t).tolist()
        new_target_pose[:4] = openravepy.quatMultiply(
          new_target_pose[:4], openravepy.quatFromAxisAngle((0, np.pi/2, 0))).tolist()

        saved_env = EnvManager.save_openrave_state(self.env)
        utils.remove_movable_objects(self.env, set())
        motion_planner_wrapper = MotionPlannerWrapper(self.env)
        traj, _, _ = motion_planner_wrapper.traj_from_pose(
          {self.manip: new_target_pose},
          collisionfree=False, n_steps=2)
        EnvManager.restore_openrave_state(self.env, saved_env)

        generic_traj = GenericTraj(self.robot, self.pr2, traj.tolist(), speed_factor=0.1)
        generic_traj.execute(sim_only=sim_only)

        self.execute(sim_only=sim_only)
      else:
        utils.grab_helper(self.robot, self.obj, self.manip)
        print("Grab succeeded!")
    else:
      utils.grab_helper(self.robot, self.obj, self.manip)


class ReleaseEvent(TrajEvent):
  def __init__(self, robot, pr2, obj, table, manip):
    super(ReleaseEvent, self).__init__(robot, pr2)
    self.obj = obj
    self.table = table
    self.manip = manip

  def execute(self, sim_only=True, use_pr2=False):
    utils.release_helper(self.robot, self.obj, self.manip)
    if self.table is not None:
      if self.table.GetName() == 'tray':
        EnvManager.tray_stack.append(self.obj.GetName())
        utils.putdown_helper(
          self.robot, self.obj, self.table,
          offset=EnvManager.tray_stack_height)
        EnvManager.tray_stack_height += 0.035
      else:
        utils.putdown_helper(self.robot, self.obj, self.table)

    if self.obj.GetName() == 'tray':
      offset = 0
      for obj in EnvManager.tray_stack:
        obj = self.env.GetKinBody(obj)
        utils.release_helper(self.robot, obj, self.manip)
        utils.putdown_helper(self.robot, obj, self.obj, offset=offset)
        offset += 0.035
      utils.include_all_collisions(self.env, EnvManager.tray_stack + ['tray'])
      
    if self.obj.GetName() == 'drawer':
      open_amount = EnvManager.drawer_init_x - self.obj.GetTransform()[0, 3]
      if open_amount > 0.0001:
        EnvManager.drawer_open_dist = open_amount
        print("Drawer opened to %f" %EnvManager.drawer_open_dist)

    if not sim_only and use_pr2:
      print("Opening {} gripper...").format(self.manip)
      if self.manip == 'rightarm':
        self.pr2.rgrip.open()
      elif self.manip == 'leftarm':
        self.pr2.lgrip.open()
      self.pr2.join_all()
      print("Opening gripper complete!")


class HandoffEvent(TrajEvent):
  def __init__(self, robot, pr2, obj, object_manip, grasping_manip, value=0.54):
    TrajEvent.__init__(self, robot, pr2)
    self.obj = obj
    self.object_manip = object_manip
    self.grasping_manip = grasping_manip
    self.value = value

  def execute(self, sim_only=True, use_pr2=False):
    if not sim_only and use_pr2:
      print("Performing handoff")

      self._iterative_grasp(self.grasping_manip, open_amount=self.value)

      print("Opening {} gripper...").format(self.object_manip)
      if self.object_manip == "rightarm":
        self.pr2.rgrip.open(scale_factor=self.value / 0.54)
      elif self.object_manip == "leftarm":
        self.pr2.lgrip.open(scale_factor=self.value / 0.54)
      self.pr2.join_all()

    utils.handoff_helper(self.robot, self.obj, self.object_manip, self.grasping_manip, self.value)
