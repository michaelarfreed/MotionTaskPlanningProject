import openravepy
import numpy as np
import json
import utils
import time

GRASPS_FILE_NAME = 'generated_grasps.json'


class GraspPoseGenerator(object):
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    # with open(GRASPS_FILE_NAME, 'r') as grasps_file:
    #   self.pregenerated_grasps = np.array(json.loads(grasps_file.read()))
    self.pregenerated_grasps = utils.side_cylinder_pre_grasps

  def generate_poses(self, obj,
                     use_general_grasps=True,
                     approach_dist=0.15):
    """
    Parameters:
    
    Returns:
    Returns a list of tuples (grasp_pose, pre_grasp_pose)
    Where grasp_pose and pre_grasp_pose are both 4x4 transformation matrices
    """    

    class _GraspOptions(object):
      def __init__(self):
        self.normalanglerange = 0.0
        self.standoffs = [0]
        self.rolls = np.arange(0.5*np.pi, 2*np.pi, np.pi)
        self.boxdelta = 0.01
        self.directiondelta = approach_dist

    gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)

    if use_general_grasps:
      gmodel.grasps = self.pregenerated_grasps
    else:
      if not gmodel.load():
        openravepy.raveLogInfo("Generating grasping model...")
        gmodel.autogenerate(_GraspOptions())

        # only use horizontal grasps
        horiz_grasp_filter = lambda g: \
          abs(gmodel.getGlobalApproachDir(g)[2]) < 0.01
        gmodel.grasps = filter(horiz_grasp_filter, gmodel.grasps)

        # only use grasps in the upper half (+y is up)
        upper_grasp_filter = lambda g: \
          gmodel.GetLocalGraspTransform(g, collisionfree=True)[1][3] > 0
        gmodel.grasps = filter(upper_grasp_filter, gmodel.grasps)

        data = json.dumps([g.tolist() for g in gmodel.grasps])
        with open(GRASPS_FILE_NAME, 'w') as outfile:
          outfile.write(data)

    # only use grasps in the upper half (+y is up)
    upper_grasp_filter = lambda g: \
      gmodel.GetLocalGraspTransform(g, collisionfree=True)[1][3] > 0.03
    gmodel.grasps = filter(upper_grasp_filter, gmodel.grasps)

    openravepy.raveLogInfo("Generating grasps")
    validgrasps, _ = gmodel.computeValidGrasps(checkcollision=False, 
                                               checkik=False,
                                               checkgrasper=False)
    np.random.shuffle(validgrasps)
    
    openravepy.raveLogInfo("Number of valid grasps: %d" % len(validgrasps))

    grasp_pose_list = []
    for grasp in validgrasps:
      grasp_pose = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)

      pre_grasp_pose = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)
      approach = gmodel.getGlobalApproachDir(grasp) * approach_dist
      pre_grasp_pose[0][3] -= approach[0]
      pre_grasp_pose[1][3] -= approach[1]
      pre_grasp_pose[2][3] -= approach[2]

      grasp_pose_list.append((grasp_pose, pre_grasp_pose))

    return grasp_pose_list


class GraspPoseGenerator2(object):
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.num_grasps = 16
    self.height_offset = 0.02
    self.dist_offset = 0.02
    self.seed = int(time.time())

  def generate_poses(self, obj, approach_dist=0.15):
    """
    Parameters:

    Returns:
    Returns a list of tuples (grasp_pose, pre_grasp_pose)
    Where grasp_pose and pre_grasp_pose are both 4x4 transformation matrices
    """
    t1 = obj.GetTransform()

    grasp_pose_list = []
    for i in range(self.num_grasps):
      rot_ang = i * (2 * np.pi) / self.num_grasps

      r1 = openravepy.matrixFromQuat((0.7071, 0, -0.7071, 0))
      t2 = openravepy.matrixFromPose((1, 0, 0, 0, -self.dist_offset, 0, 0))
      r2 = openravepy.matrixFromAxisAngle((0, 0, rot_ang))
      t3 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, self.height_offset))
      grasp_pose = t1.dot(t3).dot(r2).dot(t2).dot(r1)

      t4 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -approach_dist))
      pre_grasp_pose = grasp_pose.dot(t4)

      grasp_pose_list.append((grasp_pose, pre_grasp_pose))

    state = np.random.RandomState(self.seed)
    state.shuffle(grasp_pose_list)
    return grasp_pose_list
