import numpy as np
import openravepy
import utils
from openrave_tests.PlannerPR2 import PlannerPR2
from openrave_tests.PR2 import mirror_arm_joints, Arm
from trajectory_generator import TrajectoryGenerator, PickTrajGenerator
from grasp_pose_generator import GraspPoseGenerator2


class ObjectMover(object):
  def __init__(self, env, use_ros, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.traj_cache = {}
    self.use_ros = use_ros
    self.unmovable_objects = unmovable_objects
    self.grasp_pose_generator = GraspPoseGenerator2(self.env)
    self.traj_generator = TrajectoryGenerator(self.env)
    self.pick_traj_generator = PickTrajGenerator(self.env,
      unmovable_objects)
    if self.use_ros:
      self.pr2 = PlannerPR2(self.robot)

  def clear_cache(self):
    self.traj_cache = {}

  def pickup(self, obj):
    self.robot.SetActiveDOFs(np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
                                   self.robot.GetManipulator("leftarm").GetArmIndices()])

    # open grippers
    self.robot.SetDOFValues([0.54, 0.54],
      [self.robot.GetJointIndex('l_gripper_l_finger_joint'),
       self.robot.GetJointIndex('r_gripper_l_finger_joint')])
    if self.use_ros:
      self.pr2.rgrip.open()
      self.pr2.lgrip.open()

    # always start at same place
    left_joints = Arm.L_POSTURES['side']
    right_joints = mirror_arm_joints(left_joints).tolist()
    traj, _, _ = self.traj_generator.traj_from_joints(right_joints + left_joints)
    self._execute_traj(traj)

    # trajectory to grasp and lift
    trajs, manip = self._test_and_get_picking_trajs(obj)
    self._execute_traj(trajs[0])            # pregrasp
    self._execute_traj(trajs[1], speed=0.2) # grasp

    # close gripper
    self.robot.SetActiveManipulator(manip)
    self.robot.Grab(obj)
    utils.exclude_robot_grabbed_collisions(self.robot, obj)
    if self.use_ros:
      if manip == 'rightarm':
        self.pr2.rgrip.close()
      elif manip == 'leftarm':
        self.pr2.lgrip.close()

    self._execute_traj(trajs[2], speed=0.5) # lift


  def drop(self, obj, table):
    manip = self.robot.GetActiveManipulator().GetName()

    pos1 = [0.0, -0.7, 1.0]
    rot_y = [0.7071, 0, 0.7071, 0]
    rot = openravepy.quatMultiply(rot_y, (0, 0, 0, 1)).tolist()
    if manip == 'leftarm':
      pos1[1] *= -1

    traj1, _, _= self.traj_generator.traj_from_pose(pos1, rot, manip=manip)
    self._execute_traj(traj1.tolist())

    # with self.env:
    #   # saving values
    #   orig_values = self.robot.GetDOFValues(
    #     self.robot.GetActiveManipulator().GetArmIndices())
    #   self.robot.SetDOFValues(traj1[-1],
    #     self.robot.GetActiveManipulator().GetArmIndices())
    #   pos2 = [0.0, -0.7, 1.0]
    #   traj2, _ = self.traj_generator.traj_from_pose(pos2, rot)
    #   # reset
    #   self.robot.SetDOFValues(orig_values,
    #     self.robot.GetActiveManipulator().GetArmIndices())

    # self._execute_traj(traj1.tolist() + traj2.tolist())

    # open gripper
    self.robot.Release(obj)
    if self.use_ros:
      if manip == 'rightarm':
        self.pr2.rgrip.open()
      elif manip == 'leftarm':
        self.pr2.lgrip.open()

    # transforming the object
    T = obj.GetTransform()
    rot_angle = (np.pi / 2., 0., 0) #got this from the model
    rot_mat = openravepy.rotationMatrixFromAxisAngle(rot_angle)
    T[:3, :3] = rot_mat
    _, _, _, _, z = utils.get_object_limits(table)
    T[2, 3] = z
    obj.SetTransform(T)

  def _execute_traj(self, traj, speed=1.0):
    print("Executing trajectory...")
    utils.run_trajectory(self.robot, traj)
    if self.use_ros:
      traj_r = []
      traj_l = []
      for joints in traj:
        traj_r.append(joints[:7])
        traj_l.append(joints[7:])
      self.pr2.rarm.follow_joint_trajectory(traj_r, speed)
      self.pr2.larm.follow_joint_trajectory(traj_l, speed)
      self.pr2.join_all()
    print("Trajectory execution complete!")

  def _test_and_get_picking_trajs(self, obj_to_grasp):
    """
    Finds a valid picking trajectory or raises an ObjectMoveError
    if a valid trajectory cannot be found

    Parameters:
    obj_to_grasp: Object for which to compute a picking trajectory
    
    Returns:
    A 14-DOF (rightarm + leftarm) trajectory represented as an array of arrays
    """
    obj_name = obj_to_grasp.GetName()

    trajs, manip = self.traj_cache.get(obj_name, (None, None))
    if trajs is not None:
      print "Using existing traj in cache!"
      opt_trajs, cols, manip = self.pick_traj_generator.optimize_picking_trajs(trajs, manip, obj_to_grasp)
      if not cols:
        return opt_trajs, manip
      else:
        del self.traj_cache[obj_name]

    grasp_pose_list = self.grasp_pose_generator.generate_poses(obj_to_grasp)

    print "Trying to find a collision-free trajectory..."
    trajs, _, manip = self.pick_traj_generator.temp_generate_picking_trajs(
      obj_to_grasp, grasp_pose_list)

    if trajs is not None:
      print "Found a collision-free trajectory!!"
      return trajs, manip
    print "No collision-free trajectory found!"

    print "Trying to find any trajectory..."
    trajs, collisions, manip = self.pick_traj_generator.temp_generate_picking_trajs(
      obj_to_grasp, grasp_pose_list, collisionfree=False)

    if trajs is not None:
      print "Trajectory found with collisions: {}".format(collisions)
      self.traj_cache[obj_name] = (trajs, manip)
      e = ObjectMoveError()
      e.collision_list = [obj.GetName() for obj in collisions]
      raise e

    print "Object cannot be moved!"
    raise


class ObjectMoveError(Exception):
  pass
