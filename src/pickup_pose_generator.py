import openravepy
import numpy as np
import utils
from settings import seed, PRINT_GEN_COUNT

class PickupPoseGenerator(object):
  state = np.random.RandomState(seed)

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.num_grasps = 8
    self.height_offset = 0.02
    self.dist_offset = 0.02
    self.approach_dist = 0.15
    self.lift_dist = 0.2

  @classmethod
  def get_world_frame_pose(self, pose, loc_t):
    return loc_t.dot(pose)

  def generate_poses(self, _unused_obj):
    # generate poses in the object frame
    pose_list = []
    for i in range(self.num_grasps):
      rot_ang = i * (2 * np.pi) / self.num_grasps

      r0 = openravepy.matrixFromQuat((0, 0, 0, 1))
      r1 = openravepy.matrixFromQuat(openravepy.quatFromAxisAngle((0, -np.pi/2, 0)))
      t2 = openravepy.matrixFromPose((1, 0, 0, 0, -self.dist_offset, 0, 0))
      r2 = openravepy.matrixFromAxisAngle((0, 0, rot_ang))
      t3 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, self.height_offset))
      grasp_pose = t3.dot(r2).dot(t2).dot(r1).dot(r0)

      t4 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -self.approach_dist))
      pre_grasp_pose = grasp_pose.dot(t4)

      t5 = openravepy.matrixFromPose((1, 0, 0, 0, -self.lift_dist, 0, 0))
      lift_pose = grasp_pose.dot(t5)

      pose_list.append((pre_grasp_pose, grasp_pose, lift_pose))

    PickupPoseGenerator.state.shuffle(pose_list)
    count = 0
    for pose in pose_list:
      if PRINT_GEN_COUNT:
        print("pickup gen: " + repr(count))
        count += 1
      yield pose
