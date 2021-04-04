import openravepy
import utils
import numpy as np
import IPython
from collision_checker import CollisionChecker
from grasp_pose_generator import GraspPoseGenerator2
from putdown_pose_generator import PutdownPoseGenerator
from base_pose_generator import BasePoseGenerator
from base_around_pose_generator import BaseAroundPoseGenerator
from motion_planner import TrajoptPlanner
from rapprentice.kinematics_utils import closer_ang
from rapprentice import PR2


class ReplanError(Exception):
  pass


class TrajectoryGenerator(object):
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

  def traj_from_base_pose(self, pos, rot,
                          collisionfree=True,
                          init_dofs=None,
                          n_steps=None,
                          init_data=None):
    self.robot.SetActiveDOFs(
      np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
            self.robot.GetManipulator("leftarm").GetArmIndices()],
      openravepy.DOFAffine.X + openravepy.DOFAffine.Y + openravepy.DOFAffine.RotationAxis,
      [0, 0, 1])

    if init_dofs is None:
      init_dofs = self.robot.GetActiveDOFValues().tolist()
      init_dofs[-3] = pos[0]
      init_dofs[-2] = pos[1]
      init_dofs[-1] = openravepy.axisAngleFromQuat(rot)[2]

    # get target z within +/- pi of robot_z
    rot_i = self.robot.GetAffineDOFIndex(openravepy.DOFAffine.RotationAxis)
    robot_z = self.robot.GetActiveDOFValues()[rot_i]
    init_dofs[-1] = closer_ang(init_dofs[-1], robot_z)

    saved_env = EnvManager.save_openrave_state(self.env)
    if not collisionfree:
      utils.remove_movable_objects(self.env, self.unmovable_objects)
    traj, cost = self.motion_planner.plan_with_pose(
      pos, rot, collisionfree=collisionfree, dof_targets=init_dofs,
      n_steps=n_steps, manip='base', init_data=init_data)
    EnvManager.restore_openrave_state(self.env, saved_env)
    collisions = self.collision_checker.get_traj_collisions(traj)

    return traj, cost, collisions

  def traj_from_pose(self, pos, rot,
                     collisionfree=True,
                     init_dofs=None,
                     n_steps=None,
                     manip='rightarm',
                     init_data=None):
    """
    Returns a 14-DOF (rightarm + leftarm) trajectory to get a chosen
    gripper to a given pose.
    """
    self.robot.SetActiveDOFs(
      np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
            self.robot.GetManipulator("leftarm").GetArmIndices()])

    # find IK for init if None
    if init_dofs is None:
      target_t = openravepy.matrixFromPose(rot + pos)
      manip_to_use = self.robot.GetManipulator(manip)
      saved_env = EnvManager.save_openrave_state(self.env)
      if not collisionfree:
        utils.remove_movable_objects(self.env, self.unmovable_objects)
      init_dofs = manip_to_use.FindIKSolution(
        target_t,
        openravepy.IkFilterOptions.CheckEnvCollisions)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if init_dofs is not None:
        init_dofs = init_dofs.tolist()

    if init_dofs is not None:
      init_dofs = utils.extend_dofs(self.robot, init_dofs, manip)

      # some joints can have infinite rotation. mod to within +/- pi
      active_dof_values = self.robot.GetActiveDOFValues()
      for i, init_dof in enumerate(init_dofs):
        init_dofs[i] = closer_ang(init_dof, active_dof_values[i])

    # quaternions are rotated by pi/2 around y for some reason...
    rot = openravepy.quatMultiply(rot, (0.7071, 0, -0.7071, 0)).tolist()

    saved_env = EnvManager.save_openrave_state(self.env)
    if not collisionfree:
      utils.remove_movable_objects(self.env, self.unmovable_objects)
    traj, cost = self.motion_planner.plan_with_pose(
      pos, rot, collisionfree=collisionfree, dof_targets=init_dofs,
      n_steps=n_steps, manip=manip, init_data=init_data)
    traj = utils.extend_traj_dofs(self.robot, traj, manip)
    EnvManager.restore_openrave_state(self.env, saved_env)
    collisions = self.collision_checker.get_traj_collisions(traj)

    return traj, cost, collisions

  def traj_from_dofs(self, dof_targets,
                     collisionfree=True, n_steps=None, init_data=None):
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
      utils.remove_movable_objects(self.env, self.unmovable_objects)
    traj, cost = self.motion_planner.plan_with_dofs(
      dof_targets=dof_targets, collisionfree=collisionfree,
      n_steps=n_steps, init_data=init_data)
    EnvManager.restore_openrave_state(self.env, saved_env)
    collisions = self.collision_checker.get_traj_collisions(traj)

    return traj, cost, collisions

  # def optimize_traj(self, init_traj, collisionfree=True, manip='rightarm'):
  #   """
  #   Takes a given trajectory and optimizes it, keeping the end gripper pose
  #   the same for the chosen manipulator.

  #   If the motion planner does not support optimizing trajectories, just
  #   returns the original trajectory.
  #   """
  #   try:
  #     traj, cost = self.motion_planner.optimize_traj(
  #       init_traj, manip, collisionfree=collisionfree)
  #   except NotImplementedError:
  #     print "Motion planner does not support optimizng trajectories!"
  #   collisions = self.collision_checker.get_traj_collisions(traj)
  #   if collisionfree and collisions:
  #     return None, float('inf'), collisions
  #   else:
  #     return traj, cost, collisions

  def replan(self, init_with_old, old_action_args):
    init_data = old_action_args[0] if init_with_old else None
    traj_args_list = self._get_traj_args_list(*self.gen_args)
    args = traj_args_list[self.saved_index]
    self.saved_args = args
    traj, cost, col = self._get_traj(args, True,
                                     init_data=init_data,
                                     replan=True)
    if traj is None:
      print "Replan Error!"
      IPython.embed()  # for debugging
      raise ReplanError()
    if col != set():
      print "WARNING: Collisions during replanning: {}".format(
        [obj.GetName() for obj in col])
    return traj, cost, col

  def get_generator(self, *args):
    self.gen_args = args
    traj_args_list = self._get_traj_args_list(*args)

    print "Finding col-free {} traj...".format(self.traj_name)
    untried_traj_args_list = []
    for i, args in enumerate(traj_args_list):
      traj, cost, col = self._get_traj(args, True)
      if traj is not None and col == set():
        print "Col-free {} traj found!".format(self.traj_name)
        self.saved_index = i
        yield traj, cost, col
        self.saved_index = None
      else:
        untried_traj_args_list.append(args)

    print "Finding any {} traj...".format(self.traj_name)
    for args in untried_traj_args_list:
      traj, cost, col = self._get_traj(args, False)
      if traj is not None and col == set():
        # this should never occur. for debugging
        IPython.embed()
        continue  # temporary fix
      if traj is not None and self._collisions_okay(col):
        print "{} traj found!".format(self.traj_name)
        yield traj, cost, col

  def _collisions_okay(self, col):
    return col.intersection(self.unmovable_objects) == set()


class BaseTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set()):
    self.traj_name = "BASE"
    self.base_pose_generator = BasePoseGenerator(env)
    self.saved_index = None
    super(BaseTrajGenerator, self).__init__(env, unmovable_objects)

  def _get_traj_args_list(self, pos):
    traj_args_list = self.base_pose_generator.generate_poses(pos)
    return traj_args_list

  def get_saved_pose(self):
    return self.saved_index

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    base_pose = args

    pose = openravepy.poseFromMatrix(base_pose).tolist()
    rot = pose[:4]
    pos = pose[4:]

    with self.env:
      # check for collisions
      saved_env = EnvManager.save_openrave_state(self.env)
      self.robot.SetTransform(openravepy.matrixFromPose(rot + pos))
      collisions = self.collision_checker.get_collisions()
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)

      if collisions and not replan:
        return None, float('inf'), collisions

      # find traj
      saved_env = EnvManager.save_openrave_state(self.env)
      traj, cost, collisions = self.traj_from_base_pose(
        pos, rot, collisionfree=collisionfree, init_data=init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj is None:
        return None, float('inf'), set()

      return traj, cost, collisions


class BaseAroundTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set()):
    self.traj_name = "BASE AROUND"
    self.base_around_pose_generator = BaseAroundPoseGenerator(env)
    self.saved_index = None
    super(BaseAroundTrajGenerator, self).__init__(env, unmovable_objects)

  def get_saved_pose(self):
      return self.saved_args

  def _get_traj_args_list(self, pos):
    traj_args_list = self.base_around_pose_generator.generate_poses(pos)
    return traj_args_list

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    base_pose = args

    pose = openravepy.poseFromMatrix(base_pose).tolist()
    rot = pose[:4]
    pos = pose[4:]

    with self.env:
      # check for collisions
      saved_env = EnvManager.save_openrave_state(self.env)
      self.robot.SetTransform(openravepy.matrixFromPose(rot + pos))
      collisions = self.collision_checker.get_collisions()
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)

      if collisions and not replan:
        return None, float('inf'), collisions

      # find traj
      saved_env = EnvManager.save_openrave_state(self.env)
      traj, cost, collisions = self.traj_from_base_pose(
        pos, rot, collisionfree=collisionfree, init_data=init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj is None:
        return None, float('inf'), set()

      return traj, cost, collisions


class BaseGraspTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set()):
    self.traj_name = "BASE GRASP"
    self.base_pose_generator = BasePoseGenerator(env)
    self.grasp_pose_generator = GraspPoseGenerator2(env)
    self.saved_index = None
    super(BaseGraspTrajGenerator, self).__init__(env, unmovable_objects)

  def get_saved_pose(self):
    return self.saved_args[0]

  def _get_traj_args_list(self, obj, manip):
    obj_pose = openravepy.poseFromMatrix(obj.GetTransform()).tolist()
    base_poses = self.base_pose_generator.generate_poses(obj_pose[4:])
    traj_args_list = [(base_pose, obj, manip) for base_pose in base_poses]
    return traj_args_list

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    base_pose, obj, manip = args

    pose = openravepy.poseFromMatrix(base_pose).tolist()
    rot = pose[:4]
    pos = pose[4:]

    with self.env:
      manip_to_use = self.robot.GetManipulator(manip)

      # check for collisions
      saved_env = EnvManager.save_openrave_state(self.env)
      self.robot.SetTransform(openravepy.matrixFromPose(rot + pos))
      collisions = self.collision_checker.get_collisions()
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)

      if collisions and not replan:
        return None, float('inf'), collisions

      for grasp_pose, pre_grasp_pose in self.grasp_pose_generator.generate_poses(obj):
        # check for pre grasp ik
        saved_env = EnvManager.save_openrave_state(self.env)
        self.robot.SetTransform(openravepy.matrixFromPose(rot + pos))
        utils.remove_movable_objects(self.env, self.unmovable_objects)
        pre_grasp_ik = manip_to_use.FindIKSolution(
          pre_grasp_pose,
          openravepy.IkFilterOptions.CheckEnvCollisions)
        # reset
        EnvManager.restore_openrave_state(self.env, saved_env)
        if pre_grasp_ik is None:
          continue

        # check for grasp ik
        saved_env = EnvManager.save_openrave_state(self.env)
        self.robot.SetTransform(openravepy.matrixFromPose(rot + pos))
        utils.remove_movable_objects(self.env, self.unmovable_objects)
        grasp_ik = manip_to_use.FindIKSolution(
          grasp_pose,
          openravepy.IkFilterOptions.CheckEnvCollisions)
        # reset
        EnvManager.restore_openrave_state(self.env, saved_env)
        if grasp_ik is not None:
          break
      else:
        if not replan:
          return None, float('inf'), set()

      # find traj
      saved_env = EnvManager.save_openrave_state(self.env)
      traj, cost, collisions = self.traj_from_base_pose(
        pos, rot, collisionfree=collisionfree, init_data=init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj is None:
        return None, float('inf'), set()

      return traj, cost, collisions


class GraspTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set()):
    self.traj_name = "GRASP"
    self.grasp_pose_generator = GraspPoseGenerator2(env)
    self.saved_index = None
    super(GraspTrajGenerator, self).__init__(env, unmovable_objects)

  def _get_traj_args_list(self, obj, manip='rightarm'):
    self.target_obj = obj
    pose_pairs = self.grasp_pose_generator.generate_poses(obj)
    traj_args_list = [(obj, pose_pair, manip) for pose_pair in pose_pairs]
    return traj_args_list

  def _collisions_okay(self, col):
    return (super(GraspTrajGenerator, self)._collisions_okay(col) and
            self.target_obj not in col)

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    """
    Returns the tuple (trajs, cost, collisions) where:
    trajs: a list of trajectories, one for each step of the grasp.
    cost: total motion planner cost of all trajectories
    collisions: set of all objects the trajectory collides with

    Currently, there are two trajectories:
    1: trajectory from initial position to pregrasp
    2: trajectory from pregrasp to grasp
    """
    obj, pose_pair, manip = args

    with self.env:
      manip_to_use = self.robot.GetManipulator(manip)
      grasp_pose, pre_grasp_pose = pose_pair

      # find IK for pregrasp
      saved_env = EnvManager.save_openrave_state(self.env)
      if not collisionfree:
        utils.remove_movable_objects(self.env, self.unmovable_objects)
      init_dofs1 = manip_to_use.FindIKSolution(
        pre_grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if init_dofs1 is None and not replan:
        return None, float('inf'), set()

      # find IK for grasp
      saved_env = EnvManager.save_openrave_state(self.env)
      self.env.Remove(obj)
      if not collisionfree:
        utils.remove_movable_objects(self.env, self.unmovable_objects)
      init_dofs2 = manip_to_use.FindIKSolution(
        grasp_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if init_dofs2 is None and not replan:
        return None, float('inf'), set()

      # find trajectory for pregrasp
      saved_env = EnvManager.save_openrave_state(self.env)
      gripper_pose1 = openravepy.poseFromMatrix(pre_grasp_pose).tolist()
      xyz_target1 = gripper_pose1[4:]
      quat_target1 = gripper_pose1[:4]

      if init_data is not None:
        traj1_init_data = init_data[0]
        traj2_init_data = init_data[1]
      else:
        traj1_init_data = None
        traj2_init_data = None

      traj1, cost1, collisions1 = self.traj_from_pose(
        xyz_target1, quat_target1,
        collisionfree=collisionfree, init_dofs=init_dofs1,
        manip=manip, init_data=traj1_init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj1 is None:
        return None, float('inf'), set()

      # find trajectory to grasp
      saved_env = EnvManager.save_openrave_state(self.env)
      self.env.Remove(obj)
      utils.update_robot(self.robot, traj1[-1])

      gripper_pose2 = openravepy.poseFromMatrix(grasp_pose).tolist()
      xyz_target2 = gripper_pose2[4:]
      quat_target2 = gripper_pose2[:4]

      traj2, cost2, collisions2 = self.traj_from_pose(
        xyz_target2, quat_target2,
        collisionfree=collisionfree, init_dofs=init_dofs2,
        manip=manip, n_steps=2, init_data=traj2_init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj2 is None:
        return None, float('inf'), set()

      collisions = collisions1.union(collisions2)
      return ([traj1, traj2], cost1 + cost2, collisions)


class LiftTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set(), lift_amount=0.2):
    self.traj_name = "LIFT"
    self.lift_amount = lift_amount
    self.saved_index = None
    super(LiftTrajGenerator, self).__init__(env, unmovable_objects)

  def _get_traj_args_list(self, obj, manip='rightarm'):
    traj_args_list = [(obj, manip)]
    return traj_args_list

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    """
    Returns the tuple (traj, cost, collisions) where:
    traj: trajectory for lifting
    cost: total motion planner cost
    collisions: set of all objects the trajectory collides with
    """
    obj, manip = args

    with self.env:
      if manip == 'rightarm':
        link = 'r_gripper_tool_frame'
      elif manip == 'leftarm':
        link = 'l_gripper_tool_frame'

      lift_t = self.robot.GetLink(link).GetTransform()
      lift_pose = openravepy.poseFromMatrix(lift_t)
      lift_rot = lift_pose[:4]
      lift_rot = openravepy.quatMultiply(lift_rot, (0.7071, 0, 0.7071, 0))
      lift_pos = lift_pose[4:]
      lift_pos[2] += self.lift_amount
      lift_t = openravepy.matrixFromPose(lift_rot.tolist() + lift_pos.tolist())

      # find trajectory to lift
      gripper_pose = openravepy.poseFromMatrix(lift_t).tolist()
      xyz_target = gripper_pose[4:]
      quat_target = gripper_pose[:4]

      saved_env = EnvManager.save_openrave_state(self.env)
      traj, cost, collisions = self.traj_from_pose(
        xyz_target, quat_target, collisionfree=collisionfree, manip=manip,
        n_steps=2, init_data=init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj is None:
        return None, float('inf'), set()

      return traj, cost, collisions


class DropTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set()):
    self.traj_name = "DROP"
    self.saved_index = None
    super(DropTrajGenerator, self).__init__(env, unmovable_objects)

  def _get_traj_args_list(self, obj, manip='rightarm'):
    traj_args_list = [(obj, manip)]
    return traj_args_list

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    """
    Returns the tuple (traj, cost, collisions) where:
    traj: trajectory for dropping
    cost: total motion planner cost
    collisions: set of all objects the trajectory collides with
    """
    obj, manip = args

    with self.env:
      pos = [0.0, -0.8, 1.0]
      rot_y = [0.7071, 0, 0.7071, 0]
      rot = openravepy.quatMultiply(rot_y, (0, 0, 0, 1)).tolist()

      traj, cost, collisions = self.traj_from_pose(
        pos, rot, collisionfree=collisionfree, manip=manip, init_data=init_data)

      if traj is None:
        return None, float('inf'), set()

      return traj, cost, collisions


class PutdownTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set()):
    self.traj_name = "PUTDOWN"
    self.putdown_pose_generator = PutdownPoseGenerator(env)
    self.saved_index = None
    super(PutdownTrajGenerator, self).__init__(env, unmovable_objects)

  def _get_traj_args_list(self, obj, table, manip='rightarm'):
    pose_pairs = self.putdown_pose_generator.generate_poses(obj, table, manip=manip)
    traj_args_list = [(obj, pose_pair, manip) for pose_pair in pose_pairs]
    return traj_args_list

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    """
    Returns the tuple (trajs, cost, collisions) where:
    trajs: a list of trajectories, one for each step of the grasp.
    cost: total motion planner cost of all trajectories
    collisions: set of all objects the trajectory collides with

    Currently, there are two trajectories:
    1: trajectory from initial position to putdown
    2: trajectory from putdown to retreat
    """
    obj, pose_pair, manip = args

    with self.env:
      manip_to_use = self.robot.GetManipulator(manip)
      pose, post_pose = pose_pair

      # find IK for putdown
      saved_env = EnvManager.save_openrave_state(self.env)
      if not collisionfree:
        utils.remove_movable_objects(self.env, self.unmovable_objects)
      init_dofs1 = manip_to_use.FindIKSolution(
        pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if init_dofs1 is None and not replan:
        return None, float('inf'), set()

      # find IK for post pose
      saved_env = EnvManager.save_openrave_state(self.env)
      if not collisionfree:
        utils.remove_movable_objects(self.env, self.unmovable_objects)
      init_dofs2 = manip_to_use.FindIKSolution(
        post_pose, openravepy.IkFilterOptions.CheckEnvCollisions)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if init_dofs2 is None and not replan:
        return None, float('inf'), set()

      # find trajectory for putdown
      saved_env = EnvManager.save_openrave_state(self.env)
      gripper_pose1 = openravepy.poseFromMatrix(pose).tolist()
      xyz_target1 = gripper_pose1[4:]
      quat_target1 = gripper_pose1[:4]

      if init_data is not None:
        traj1_init_data = init_data[0]
        traj2_init_data = init_data[1]
      else:
        traj1_init_data = None
        traj2_init_data = None

      traj1, cost1, collisions1 = self.traj_from_pose(
        xyz_target1, quat_target1,
        collisionfree=collisionfree, init_dofs=init_dofs1,
        manip=manip, init_data=traj1_init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj1 is None:
        return None, float('inf'), set()

      # find trajectory for retreat
      saved_env = EnvManager.save_openrave_state(self.env)
      utils.update_robot(self.robot, traj1[-1])
      self.env.Remove(obj)

      gripper_pose2 = openravepy.poseFromMatrix(post_pose).tolist()
      xyz_target2 = gripper_pose2[4:]
      quat_target2 = gripper_pose2[:4]

      traj2, cost2, collisions2 = self.traj_from_pose(
        xyz_target2, quat_target2,
        collisionfree=collisionfree, init_dofs=init_dofs2,
        manip=manip, n_steps=2, init_data=traj2_init_data)
      # reset
      EnvManager.restore_openrave_state(self.env, saved_env)
      if traj2 is None:
        return None, float('inf'), set()

      collisions = collisions1.union(collisions2)
      return ([traj1, traj2], cost1 + cost2, collisions)


class PostureTrajGenerator(TrajectoryGenerator):
  def __init__(self, env, unmovable_objects=set(), lift_amount=0.2):
    self.traj_name = "POSTURE"
    self.lift_amount = lift_amount
    self.saved_index = None
    super(PostureTrajGenerator, self).__init__(env, unmovable_objects)

  def _get_traj_args_list(self, posture):
    traj_args_list = [posture]
    return traj_args_list

  def _get_traj(self, args, collisionfree, init_data=None, replan=False):
    """
    Returns the tuple (traj, cost, collisions) where:
    traj: trajectory to the chosen posture
    cost: total motion planner cost
    collisions: set of all objects the trajectory collides with
    """
    posture = args

    with self.env:
      left_joints = PR2.Arm.L_POSTURES[posture]
      right_joints = PR2.mirror_arm_joints(left_joints).tolist()
      traj, cost, collisions = self.traj_from_dofs(
        right_joints + left_joints,
        collisionfree=collisionfree, init_data=init_data)
      if traj is None:
        return None, float('inf'), set()

      return traj, cost, collisions
