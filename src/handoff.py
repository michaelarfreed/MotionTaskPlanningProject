from rapprentice.PR2 import PR2, Arm, mirror_arm_joints
from lip_pickup_pose_generator import LipPickupPoseGenerator
from operator import add
import utils
import trajevent
from hl_action import HLAction
import numpy as np
import openravepy
from env_manager import EnvManager


class HandoffError(Exception):
  pass


class HandoffAction(HLAction):
  print_mode = False
  
  def __init__(self, robot, pr2, unmovable_objects, lineno, arm_with_object, obj_name, use_bullet=False):
    object_manip_name = utils.to_manip_better(arm_with_object)
    if object_manip_name is None:
      raise HandoffError("Invalid arm_with_object name: %s" % arm_with_object)

    HLAction.__init__(self, robot, pr2, unmovable_objects, lineno)
    self.obj = self.env.GetKinBody(obj_name)
    if not self.obj:
      raise HandoffError("Invalid obj_name: %s" % obj_name)

    self.object_manip = self.robot.GetManipulator(object_manip_name)
    grasping_manip_name = "rightarm" if object_manip_name == "leftarm" else "leftarm"
    self.grasping_manip = self.robot.GetManipulator(grasping_manip_name)

    self.pickup_pose_generator = LipPickupPoseGenerator(self.env, self.unmovable_objects)
    self.last_target_cache_key = None

    if use_bullet:
      # default collision checker doesn't catch collisions between objects
      collision_checker = openravepy.RaveCreateCollisionChecker(self.env, "bullet")
      collision_checker.SetCollisionOptions(openravepy.CollisionOptions.Contacts)
      self.env.SetCollisionChecker(collision_checker)

    self.open_gripper_val = 0.25

  def execute_handoff(self):
    # for testing purposes
    self.traj_events = None
    try:
      while self.traj_events is None:
        self.traj_events = self.get_trajevents(self._get_pickup_poses(), self.motion_planner_wrapper, fast=False)
    except StopIteration:
      print("Ran out of pickup poses, quitting")
      return
    for traj in self.traj_events:
      traj.execute(sim_only=False)

  def get_trajevents(
    self, pickup_poses, motion_planner_wrapper,
    error_free_only=True, fast=False, replan=False):
    # note: error_free_only is not used
    # finding IKs
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)

      if self.grasping_manip.GetName() == "rightarm":
        self.robot.SetActiveDOFs(self.grasping_manip.GetArmIndices().tolist() + self.object_manip.GetArmIndices().tolist())
      else:
        self.robot.SetActiveDOFs(self.object_manip.GetArmIndices().tolist() + self.grasping_manip.GetArmIndices().tolist())

      handoff_pose = utils.get_handoff_pose(self.robot, self.object_manip.GetName())
      pre_grasp_pose, grasp_pose, _ = pickup_poses

      handoff_ik = self.object_manip.FindIKSolution(
        handoff_pose, openravepy.IkFilterOptions.CheckEnvCollisions)

      if handoff_ik is None:
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find IK solution for handoff pose")
        return None
      
      utils.update_robot(self.robot, utils.extend_dofs(self.robot, handoff_ik, self.object_manip.GetName()))
      prep_ik = trajevent.FastArm(self.robot, self.pr2, handoff_ik, self.object_manip.GetName())

      rotated_handoff_pose = self._rotate_handoff_pose_if_needed(handoff_pose)
      if rotated_handoff_pose is not None:
        rotated_handoff_ik = self.object_manip.FindIKSolution(
          rotated_handoff_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        if rotated_handoff_ik is None:
          EnvManager.restore_openrave_state(self.env, saved_env)
          if HandoffAction.print_mode:
            print("Could not find IK solution for rotating object")
          return None
        
        utils.update_robot(self.robot, utils.extend_dofs(self.robot, rotated_handoff_ik, self.object_manip.GetName()))
        prep_ik = trajevent.FastArm(self.robot, self.pr2, rotated_handoff_ik, self.object_manip.GetName())

      pre_grasp_pose = self.obj.GetTransform().dot(pre_grasp_pose)
      grasp_pose = self.obj.GetTransform().dot(grasp_pose)

      self.robot.SetActiveManipulator(self.grasping_manip.GetName())
      utils.open_gripper(self.robot, self.grasping_manip.GetName(), self.open_gripper_val)

      pre_grasp_ik = self.grasping_manip.FindIKSolution(
        pre_grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      if pre_grasp_ik is None:
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find IK solution for pre-grasp pose")
        return None
      
      utils.update_robot(self.robot, utils.extend_dofs(self.robot, pre_grasp_ik, self.grasping_manip.GetName()))

      self.env.Remove(self.obj)
      grasp_ik = self.grasping_manip.FindIKSolution(
        grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      self.env.AddKinBody(self.obj)
      if grasp_ik is None:
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find IK solution for grasp pose")
        return None

      retreat_pose = utils.get_adjusted_pose(self.robot, self.object_manip.GetName(), 0.05)

      self.env.Remove(self.obj)
      retreat_ik = self.object_manip.FindIKSolution(
        retreat_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      self.env.AddKinBody(self.obj)
      if retreat_ik is None:
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find IK solution for retreat pose")
        return None

      EnvManager.restore_openrave_state(self.env, saved_env)

    if fast:
      return [prep_ik] + [trajevent.OpenGripperEvent(self.robot, self.pr2, self.grasping_manip.GetName(), self.open_gripper_val),
                          trajevent.FastArm(self.robot, self.pr2, pre_grasp_ik, self.grasping_manip.GetName()),
                          trajevent.FastArm(self.robot, self.pr2, grasp_ik, self.grasping_manip.GetName()),
                          trajevent.HandoffEvent(self.robot, self.pr2, self.obj,
                                                 self.object_manip.GetName(), self.grasping_manip.GetName(), self.open_gripper_val),
                          trajevent.FastArm(self.robot, self.pr2, retreat_ik, self.object_manip.GetName()),
                          trajevent.FastArm(self.robot, self.pr2, self._get_side_joint_dofs()[0], "rightarm"),
                          trajevent.FastArm(self.robot, self.pr2, self._get_side_joint_dofs()[1], "leftarm")]

    # motion planning
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)

      if rotated_handoff_pose is not None:
        pose_to_use = rotated_handoff_pose
        ik_to_use = rotated_handoff_ik
      else:
        pose_to_use = handoff_pose
        ik_to_use = handoff_ik
      grasping_manip_curr_pose = openravepy.poseFromMatrix(self.grasping_manip.GetTransform()).tolist()
      pose_to_use = openravepy.poseFromMatrix(pose_to_use).tolist()
      if replan:
        init_data0 = self.last_trajevents[0].traj
      else:
        init_data0 = None
      traj0, cost0, col0 = motion_planner_wrapper.traj_from_pose(
        {self.object_manip.GetName(): pose_to_use,
         self.grasping_manip.GetName(): grasping_manip_curr_pose},
        init_dofs=utils.extend_dofs(self.robot, ik_to_use, self.object_manip.GetName()),
        init_data=init_data0)
      if col0 != set():
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find collision-free traj to handoff pose")
        return None
      
      utils.update_robot(self.robot, traj0[-1])

      pre_grasp_pose = openravepy.poseFromMatrix(pre_grasp_pose).tolist()
      grasp_pose = openravepy.poseFromMatrix(grasp_pose).tolist()
      retreat_pose = openravepy.poseFromMatrix(retreat_pose).tolist()

      self.robot.SetActiveManipulator(self.grasping_manip.GetName())
      utils.open_gripper(self.robot, self.grasping_manip.GetName(), self.open_gripper_val)

      obj_manip_curr_pose = openravepy.poseFromMatrix(self.object_manip.GetTransform()).tolist()
      if replan:
        init_data1 = self.last_trajevents[2].traj
      else:
        init_data1 = None
      traj1, cost1, col1 = motion_planner_wrapper.traj_from_pose(
        {self.grasping_manip.GetName(): pre_grasp_pose,
         self.object_manip.GetName(): obj_manip_curr_pose},
        init_dofs=utils.extend_dofs(self.robot, pre_grasp_ik, self.grasping_manip.GetName()),
        init_data=init_data1)
      if col1 != set():
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find collision-free traj to pre-grasp pose")
        return None

      utils.update_robot(self.robot, traj1[-1])

      self.env.Remove(self.obj)
      obj_manip_curr_pose = openravepy.poseFromMatrix(self.object_manip.GetTransform()).tolist()
      if replan:
        init_data2 = self.last_trajevents[3].traj
      else:
        init_data2 = None
      traj2, cost2, col2 = motion_planner_wrapper.traj_from_pose(
        {self.grasping_manip.GetName(): grasp_pose,
         self.object_manip.GetName(): obj_manip_curr_pose},
        init_dofs=utils.extend_dofs(self.robot, grasp_ik, self.grasping_manip.GetName()),
        init_data=init_data2, maintain_linear=True)
      self.env.AddKinBody(self.obj)
      if col2 != set():
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find collision-free traj to grasp pose")
        return None

      utils.update_robot(self.robot, traj2[-1])
      utils.handoff_helper(self.robot, self.obj, self.object_manip.GetName(), self.grasping_manip.GetName(), self.open_gripper_val)

      grasping_manip_curr_pose = openravepy.poseFromMatrix(self.grasping_manip.GetTransform()).tolist()
      if replan:
        init_data3 = self.last_trajevents[5].traj
      else:
        init_data3 = None
      traj3, cost3, col3 = motion_planner_wrapper.traj_from_pose(
        {self.object_manip.GetName(): retreat_pose,
         self.grasping_manip.GetName(): grasping_manip_curr_pose},
        init_dofs=utils.extend_dofs(self.robot, retreat_ik, self.object_manip.GetName()),
        init_data=init_data3, maintain_linear=True)
      if col3 != set():
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find collision-free traj to retreat pose")
        return None

      utils.update_robot(self.robot, traj3[-1])

      if replan:
        init_data4 = self.last_trajevents[6].traj
      else:
        init_data4 = None
      traj4, cost4, col4 = motion_planner_wrapper.traj_from_dofs(
        reduce(add, self._get_side_joint_dofs(), []),
        init_data=init_data4)
      if col4 != set():
        EnvManager.restore_openrave_state(self.env, saved_env)
        if HandoffAction.print_mode:
          print("Could not find collision-free traj to side pose")
        return None

      EnvManager.restore_openrave_state(self.env, saved_env)

      return [trajevent.GenericTraj(self.robot, self.pr2, traj0),
              trajevent.OpenGripperEvent(self.robot, self.pr2, self.grasping_manip.GetName(), self.open_gripper_val),
              trajevent.GenericTraj(self.robot, self.pr2, traj1),
              trajevent.GenericTraj(self.robot, self.pr2, traj2, speed_factor=0.1),
              trajevent.HandoffEvent(self.robot, self.pr2, self.obj,
                                     self.object_manip.GetName(), self.grasping_manip.GetName(), self.open_gripper_val),
              trajevent.GenericTraj(self.robot, self.pr2, traj3, speed_factor=0.1),
              trajevent.GenericTraj(self.robot, self.pr2, traj4)]

  def _init_data_generator(self):
    self.target_generator = self.pickup_pose_generator.generate_poses(self.obj)

  def _get_pickup_poses(self):
    self.prep_next()
    return self.next_target

  def _get_side_joint_dofs(self):
    # side posture joints
    left_joints = utils.constrain_within_joint_limits(
      self.robot, Arm.L_POSTURES['side2'], arm='leftarm')
    right_joints = utils.constrain_within_joint_limits(
      self.robot, mirror_arm_joints(left_joints).tolist(), arm='rightarm')

    return right_joints, left_joints

  def get_name(self):
    return "handoff %s to %s" % (self.object_manip.GetName(), self.grasping_manip.GetName())

  def _rotate_handoff_pose_if_needed(self, handoff_pose):
    """ If the gripper is on the wrong side of the object, need to rotate it by pi. """
    robot_t = self.robot.GetTransform()
    obj_world_t = self.obj.GetTransform()
    obj_t = np.linalg.inv(robot_t).dot(obj_world_t)
    obj_y = obj_t[1, 3]
    gripper_world_t = self.object_manip.GetEndEffectorTransform()
    gripper_t = np.linalg.inv(robot_t).dot(gripper_world_t)
    gripper_y = gripper_t[1, 3]
    arm_with_object = self.object_manip.GetName()
    handoff = openravepy.poseFromMatrix(handoff_pose).tolist()
    if (arm_with_object == "leftarm" and obj_y > gripper_y) or (arm_with_object == "rightarm" and obj_y < gripper_y):
      rotated = [-handoff[3], handoff[2], -handoff[1], handoff[0]] + handoff[4:7]
      return openravepy.matrixFromPose(rotated)

  # def _make_object_upright(self, saved_env):
  #   traj_list = []

  #   last_error = -1
  #   obj_world_t = self.obj.GetTransform()
  #   error = np.linalg.norm(openravepy.poseFromMatrix(obj_world_t)[0:4] - [1, 0, 0, 0])
  #   while last_error == -1 or error < last_error:
  #     dofs_cache = [list(self.l_joint_dofs), list(self.r_joint_dofs)]
  #     if self.arm_with_object == "leftarm":
  #       self.l_joint_dofs[5] += 0.03
  #     else:
  #       self.r_joint_dofs[5] += 0.03
  #     try:
  #       traj_list.append(self._get_handoff_pose_event(saved_env))
  #     except HandoffError:
  #       raise HandoffError("Could not find collision-free traj to make object upright")

  #     last_error = error
  #     obj_world_t = self.obj.GetTransform()
  #     error = np.linalg.norm(openravepy.poseFromMatrix(obj_world_t)[0:4] - [1, 0, 0, 0])

  #   # last object transform caused increased error, so remove it and reset that trajectory
  #   self.l_joint_dofs, self.r_joint_dofs = dofs_cache
  #   utils.update_robot(self.robot, self._joint_dofs)
  #   return traj_list[:-1]
