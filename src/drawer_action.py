from drawer_pose_generator import DrawerPoseGenerator
from rapprentice.PR2 import PR2, Arm, mirror_arm_joints
import utils
import trajevent
from hl_action import HLAction
import numpy as np
import openravepy
from env_manager import EnvManager
import time

print_mode = False

class DrawerAction(HLAction):
  def __init__(self, robot, pr2, unmovable_objects, lineno, drawer, manip, pose_name, open):
    HLAction.__init__(self, robot, pr2, unmovable_objects, lineno)
    self.obj = drawer
    self.manip = manip
    self.pose_name = pose_name
    self.open = open
    self.pose_generator = DrawerPoseGenerator(self.env, self.unmovable_objects, self.open)
    self.last_target_cache_key = None

  def get_name(self):
    if self.open:
      return "opendrawer %s %s" %(self.obj.GetName(), self.manip)
    else:
      return "closedrawer %s %s" %(self.obj.GetName(), self.manip)

  def _init_data_generator(self):
    self.target_generator = self.pose_generator.generate_poses(self.obj)

  def get_trajevents(self, target_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    (pre_pose, open_pose_start, open_pose_end, retreat_pose) = target_poses

    pre_pose = self.pose_generator.get_world_frame_pose(pre_pose)
    open_pose_start = self.pose_generator.get_world_frame_pose(open_pose_start)
    open_pose_end = self.pose_generator.get_world_frame_pose(open_pose_end)
    retreat_pose = self.pose_generator.get_world_frame_pose(retreat_pose)

    # finding IKs
    manip_to_use = self.robot.GetManipulator(self.manip)
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      utils.open_gripper(self.robot, self.manip)

      pre_pose_ik = manip_to_use.FindIKSolution(
        pre_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      if pre_pose_ik is None and error_free_only:
        if print_mode:
          print("Could not find col-free IK solution to pre pose")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      open_pose_start_ik = manip_to_use.FindIKSolution(
        open_pose_start, openravepy.IkFilterOptions.CheckEnvCollisions)
      if open_pose_start_ik is None and error_free_only:
        if print_mode:
          print("Could not find col-free IK solution to open pose start")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None
      if open_pose_start_ik is not None:
        self.robot.SetDOFValues(
          open_pose_start_ik, self.robot.GetManipulator(self.manip).GetArmIndices())

      utils.grab_helper(self.robot, self.obj, self.manip)
      utils.open_gripper(self.robot, self.manip)  # so not in collision

      open_pose_end_ik = manip_to_use.FindIKSolution(
        open_pose_end, openravepy.IkFilterOptions.CheckEnvCollisions)
      
      if open_pose_end_ik is None and error_free_only:
        if print_mode:
          print("Could not find col-free IK solution to open pose end")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None
      if open_pose_end_ik is not None:
        self.robot.SetDOFValues(
          open_pose_end_ik, self.robot.GetManipulator(self.manip).GetArmIndices())

      utils.release_helper(self.robot, self.obj, self.manip)

      retreat_pose_ik = manip_to_use.FindIKSolution(
        retreat_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      if retreat_pose_ik is None and error_free_only:
        if print_mode:
          print("Could not find col-free IK solution to retreat pose")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      EnvManager.restore_openrave_state(self.env, saved_env)
      utils.open_gripper(self.robot, self.manip)

      excluded = [self.env.GetKinBody(o) for o in EnvManager.object_groups[self.obj.GetName()].union({self.obj.GetName()})]
      utils.remove_movable_objects(
        self.env, self.unmovable_objects, excluded=excluded)

      if pre_pose_ik is None:
        pre_pose_ik = manip_to_use.FindIKSolution(
          pre_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if pre_pose_ik is None:
          if print_mode:
            print("Could not find any IK solution to pre pose")
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      if open_pose_start_ik is None:
        open_pose_start_ik = manip_to_use.FindIKSolution(
          open_pose_start, openravepy.IkFilterOptions.CheckEnvCollisions)
        if open_pose_start_ik is None:
          if print_mode:
            print("Could not find any IK solution to open pose start")
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      self.robot.SetDOFValues(
        open_pose_start_ik, self.robot.GetManipulator(self.manip).GetArmIndices())
      
      utils.grab_helper(self.robot, self.obj, self.manip)      
      utils.open_gripper(self.robot, self.manip)  # so not in collision

      if open_pose_end_ik is None:
        open_pose_end_ik = manip_to_use.FindIKSolution(
          open_pose_end, openravepy.IkFilterOptions.CheckEnvCollisions)
        if open_pose_end_ik is None:
          if print_mode:
            print("Could not find any IK solution to open pose end")
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      self.robot.SetDOFValues(
        open_pose_end_ik, self.robot.GetManipulator(self.manip).GetArmIndices())

      utils.release_helper(self.robot, self.obj, self.manip)

      if retreat_pose_ik is None:
        retreat_pose_ik = manip_to_use.FindIKSolution(
          retreat_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if retreat_pose_ik is None:
          if print_mode:
            print("Could not find any IK solution to retreat pose")
          EnvManager.restore_openrave_state(self.env, saved_env)
          return None

      EnvManager.restore_openrave_state(self.env, saved_env)

    # side posture joints
    left_joints = utils.constrain_within_joint_limits(
      self.robot, Arm.L_POSTURES['side2'], arm='leftarm')
    right_joints = utils.constrain_within_joint_limits(
      self.robot, mirror_arm_joints(left_joints).tolist(), arm='rightarm')

    if fast:
      return [trajevent.OpenGripperEvent(self.robot, self.pr2, self.manip),
              trajevent.FastArm(self.robot, self.pr2, pre_pose_ik, self.manip),
              trajevent.FastArm(self.robot, self.pr2, open_pose_start_ik, self.manip),
              trajevent.GrabEvent(self.robot, self.pr2, self.obj, self.manip),
              trajevent.FastArm(self.robot, self.pr2, open_pose_end_ik, self.manip),
              trajevent.ReleaseEvent(self.robot, self.pr2, self.obj, table=None, manip=self.manip),
              trajevent.FastArm(self.robot, self.pr2, retreat_pose_ik, self.manip),
              trajevent.FastArm(self.robot, self.pr2, left_joints, 'leftarm'),
              trajevent.FastArm(self.robot, self.pr2, right_joints, 'rightarm')]

    # motion planning
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)

      self.robot.SetActiveManipulator(self.manip)
      utils.open_gripper(self.robot, self.manip)

      pre_pose = openravepy.poseFromMatrix(pre_pose).tolist()
      open_pose_start = openravepy.poseFromMatrix(open_pose_start).tolist()
      open_pose_end = openravepy.poseFromMatrix(open_pose_end).tolist()
      retreat_pose = openravepy.poseFromMatrix(retreat_pose).tolist()

      if replan:
        init_data0 = self.last_trajevents[1].traj
      else:
        init_data0 = None
      traj0, cost0, col0 = motion_planner_wrapper.traj_from_pose(
        {self.manip: pre_pose},
        init_dofs=utils.extend_dofs(self.robot, pre_pose_ik, self.manip),
        init_data=init_data0)
      if error_free_only and col0 != set():
        if print_mode:
          print("Motion planning to pre pose failed")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      utils.update_robot(self.robot, traj0[-1])

      self.env.Remove(self.obj)
      if replan:
        init_data1 = self.last_trajevents[2].traj
      else:
        init_data1 = None
      traj1, cost1, col1 = motion_planner_wrapper.traj_from_pose(
        {self.manip: open_pose_start},
        init_dofs=utils.extend_dofs(self.robot, open_pose_start_ik, self.manip),
        init_data=init_data1, maintain_linear=True)
      self.env.AddKinBody(self.obj)
      if error_free_only and col1 != set():
        if print_mode:
          print("Motion planning to open pose start failed")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      utils.update_robot(self.robot, traj1[-1])
      
      utils.grab_helper(self.robot, self.obj, self.manip)

      if replan:
        init_data2 = self.last_trajevents[4].traj
      else:
        init_data2 = None
      traj2, cost2, col2 = motion_planner_wrapper.traj_from_pose(
        {self.manip: open_pose_end},
        init_dofs=utils.extend_dofs(self.robot, open_pose_end_ik, self.manip),
        init_data=init_data2, maintain_linear=True)
      if error_free_only and col2 != set():
        if print_mode:
          print("Motion planning to open pose end failed")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      utils.update_robot(self.robot, traj2[-1])
      
      utils.release_helper(self.robot, self.obj, self.manip)

      if replan:
        init_data3 = self.last_trajevents[6].traj
      else:
        init_data3 = None
      traj3, cost3, col3 = motion_planner_wrapper.traj_from_pose(
        {self.manip: retreat_pose},
        init_dofs=utils.extend_dofs(self.robot, retreat_pose_ik, self.manip),
        init_data=init_data3, maintain_linear=True)
      self.env.AddKinBody(self.obj)
      if error_free_only and col3 != set():
        if print_mode:
          print("Motion planning to retreat pose failed")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      utils.update_robot(self.robot, traj3[-1])

      if replan:
        init_data4 = self.last_trajevents[7].traj
      else:
        init_data4 = None
      traj4, cost4, col4 = motion_planner_wrapper.traj_from_dofs(
        right_joints + left_joints, init_data=init_data4)
      if error_free_only and col4 != set():
        if print_mode:
          print("Motion planning to side pose failed")
        EnvManager.restore_openrave_state(self.env, saved_env)
        return None

      EnvManager.restore_openrave_state(self.env, saved_env)

      collisions = col0.union(col1).union(col2).union(col3).union(col4)
      if self.obj in collisions:
        return None
      if collisions:
        if error_free_only:
          print collisions
          return None
        else:
          self.raise_collisions_error(collisions)

      return [trajevent.OpenGripperEvent(self.robot, self.pr2, self.manip),
              trajevent.GenericTraj(self.robot, self.pr2, traj0),
              trajevent.GenericTraj(self.robot, self.pr2, traj1, speed_factor=0.1),
              trajevent.GrabEvent(self.robot, self.pr2, self.obj, self.manip),
              trajevent.GenericTraj(self.robot, self.pr2, traj2, speed_factor=0.2),
              trajevent.ReleaseEvent(self.robot, self.pr2, self.obj, table=None, manip=self.manip),
              trajevent.GenericTraj(self.robot, self.pr2, traj3, speed_factor=0.1),
              trajevent.GenericTraj(self.robot, self.pr2, traj4)]
