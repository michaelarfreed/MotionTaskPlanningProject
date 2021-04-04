import openravepy
import utils
import numpy as np
from collision_checker import CollisionChecker
from motion_planner import TrajoptPlanner
from rapprentice.kinematics_utils import closer_ang
from env_manager import EnvManager

class MotionPlannerWrapper(object):
  """
  Wrapper class around the motion planner and checks the resulting
  trajectory for collisions.
  """
  def __init__(self, env, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.motion_planner = TrajoptPlanner(self.env)
    self.collision_checker = CollisionChecker(self.env)
    self.unmovable_objects = unmovable_objects

  def traj_from_base_pose(self, pose,
                          collisionfree=True,
                          init_dofs=None,
                          n_steps=30,
                          init_data=None):
    self.robot.SetActiveDOFs(
      np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
            self.robot.GetManipulator("leftarm").GetArmIndices()],
      openravepy.DOFAffine.X + openravepy.DOFAffine.Y + openravepy.DOFAffine.RotationAxis,
      [0, 0, 1])

    if init_dofs is None:
      init_dofs = self.robot.GetActiveDOFValues().tolist()
      init_dofs[-3] = pose[4]
      init_dofs[-2] = pose[5]
      init_dofs[-1] = openravepy.axisAngleFromQuat(pose[:4])[2]

    # get target z within +/- pi of robot_z
    rot_i = self.robot.GetAffineDOFIndex(openravepy.DOFAffine.RotationAxis)
    robot_z = self.robot.GetActiveDOFValues()[rot_i]
    init_dofs[-1] = closer_ang(init_dofs[-1], robot_z)

    saved_env = EnvManager.save_openrave_state(self.env)
    if not collisionfree:
      pass
      # utils.remove_movable_objects(self.env, self.unmovable_objects)
    traj, cost = self.motion_planner.base_plan_multi_init(
      pose, init_dofs, collisionfree=collisionfree,
      n_steps=n_steps, init_data=init_data,
      maintain_rel=True, maintain_up=True)
    EnvManager.restore_openrave_state(self.env, saved_env)
    collisions = self.collision_checker.get_traj_collisions(traj)

    return traj, cost, collisions

  def traj_from_pose(self, pose_by_manip,
                     collisionfree=True,
                     init_dofs=None,
                     n_steps=30,
                     init_data=None,
                     maintain_rel=False,
                     maintain_up=False,
                     maintain_linear=False,
                     include_obj_world_col=True):
    """
    Returns a 14-DOF (rightarm + leftarm) trajectory to get a chosen
    gripper to a given pose.
    """
    self.robot.SetActiveDOFs(
      np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
            self.robot.GetManipulator("leftarm").GetArmIndices()])

    # # find IK for init if None
    # if init_dofs is None:
    #   target_t = openravepy.matrixFromPose(rot + pos)
    #   manip_to_use = self.robot.GetManipulator(manip)
    #   saved_env = EnvManager.save_openrave_state(self.env)
    #   if not collisionfree:
    #     utils.remove_movable_objects(self.env, self.unmovable_objects)
    #   try:
    #     init_dofs = manip_to_use.FindIKSolution(
    #       target_t,
    #       openravepy.IkFilterOptions.CheckEnvCollisions)
    #   except:
    #     pass
    #     # TODO: This sometimes fails.. Ignore for now.
    #   # reset
    #   EnvManager.restore_openrave_state(self.env, saved_env)
    #   if init_dofs is not None:
    #     init_dofs = init_dofs.tolist()

    # if init_dofs is not None:
    #   init_dofs = utils.extend_dofs(self.robot, init_dofs, manip)

    # some joints can have infinite rotation. mod to within +/- pi
    if init_dofs is not None:
      active_dof_values = self.robot.GetActiveDOFValues()
      for i, init_dof in enumerate(init_dofs):
        init_dofs[i] = closer_ang(init_dof, active_dof_values[i])

    for manip, pose in pose_by_manip.items():
      pose_copy = list(pose)  # make copy
      # quaternions are rotated by pi/2 around y for some reason...
      pose_copy[:4] = openravepy.quatMultiply(pose_copy[:4], openravepy.quatFromAxisAngle((0, -np.pi/2, 0))).tolist()
      pose_by_manip[manip] = pose_copy

    saved_env = EnvManager.save_openrave_state(self.env)
    if not collisionfree:
      pass
      # utils.remove_movable_objects(self.env, self.unmovable_objects)
    traj, cost = self.motion_planner.plan_with_pose(
      pose_by_manip, collisionfree=collisionfree, init_dofs=init_dofs,
      n_steps=n_steps, init_data=init_data,
      maintain_rel=maintain_rel, maintain_up=maintain_up,
      maintain_linear=maintain_linear)
    EnvManager.restore_openrave_state(self.env, saved_env)
    collisions = self.collision_checker.get_traj_collisions(
      traj, include_obj_world_col=include_obj_world_col)

    return traj, cost, collisions

  def traj_from_dofs(self, dof_targets,
                     collisionfree=True, n_steps=30, init_data=None):
    """
    Returns a 14-DOF (rightarm + leftarm) trajectory to move both arms
    to given joint positions
    """
    self.robot.SetActiveDOFs(
      np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
            self.robot.GetManipulator("leftarm").GetArmIndices()])

    # some joints can have infinite rotation. mod to within +/- pi
    active_dof_values = self.robot.GetActiveDOFValues()
    for i, dof_target in enumerate(dof_targets):
      dof_targets[i] = closer_ang(dof_target, active_dof_values[i])

    saved_env = EnvManager.save_openrave_state(self.env)
    if not collisionfree:
      pass
      # utils.remove_movable_objects(self.env, self.unmovable_objects)
    traj, cost = self.motion_planner.plan_with_dofs(
      dof_targets=dof_targets, collisionfree=collisionfree,
      n_steps=n_steps, init_data=init_data)
    EnvManager.restore_openrave_state(self.env, saved_env)
    collisions = self.collision_checker.get_traj_collisions(traj)

    return traj, cost, collisions
