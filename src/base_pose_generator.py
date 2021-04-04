import openravepy
import numpy as np
import time


class BasePoseGenerator(object):
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.num_poses_per_ring = 8
    self.dist_offset_range = np.linspace(0.5, 0.85, num=2)
    self.angle_range = [0]  # np.linspace(-np.pi/8, np.pi/8, num=3)
    self.seed = int(time.time())

  def generate_poses(self, pos):
    """
    Generates base poses in that faces the desired
    position within some angle range.
    """
    t1 = openravepy.matrixFromPose([1, 0, 0, 0] + pos)

    pose_list = []
    for dist_offset in self.dist_offset_range:
      for i in range(self.num_poses_per_ring):
        for angle in self.angle_range:
            rot_ang = i * (2 * np.pi) / self.num_poses_per_ring

            r0 = openravepy.matrixFromAxisAngle((0, 0, angle))
            r1 = openravepy.matrixFromQuat((0, 0, 0, 1))
            t2 = openravepy.matrixFromPose((1, 0, 0, 0, dist_offset, 0, 0))
            r2 = openravepy.matrixFromAxisAngle((0, 0, rot_ang))
            pose = t1.dot(r2).dot(t2).dot(r1).dot(r0)
            pose[2][3] = 0
            pose_list.append(pose)

    state = np.random.RandomState(self.seed)
    state.shuffle(pose_list)
    return pose_list
