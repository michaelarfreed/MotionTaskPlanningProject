import openravepy
import numpy as np
import utils
from settings import seed, DISABLE_BASE, PRINT_GEN_COUNT
from env_manager import EnvManager
import time

class BasePoseGenerator2(object):
  state = np.random.RandomState(seed)

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.num_poses_per_ring = 4
    self.dist_offset_range = np.linspace(0.55, 0.75, num=4)
    self.angle_range = [0]  # np.linspace(-np.pi/8, np.pi/8, num=3)

  @classmethod
  def get_world_frame_pose(self, base_pose, loc_t):
    # Actually, we do care about orientation...
    # loc_pose = openravepy.poseFromMatrix(loc_t)
    # loc_pose[:4] = (1, 0, 0, 0)  # we really only care about position
    # loc_t = openravepy.matrixFromPose(loc_pose)

    base_t = loc_t.dot(base_pose)
    base_t[2][3] = 0  # make sure base is on the ground
    return base_t

  def _calc_pose_list(self):
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      pose_list = []
      for dist_offset in self.dist_offset_range:
        for i in range(self.num_poses_per_ring):
          for angle in self.angle_range:
            rot_ang = i * (2 * np.pi) / self.num_poses_per_ring

            r0 = openravepy.matrixFromAxisAngle((0, 0, angle))
            r1 = openravepy.matrixFromQuat((0, 0, 0, 1))
            t2 = openravepy.matrixFromPose((1, 0, 0, 0, dist_offset, 0, 0))
            r2 = openravepy.matrixFromAxisAngle((0, 0, rot_ang))
            pose = r2.dot(t2).dot(r1).dot(r0)
            pose_list.append(pose)

      EnvManager.restore_openrave_state(self.env, saved_env)
      return pose_list

  def generate_gp_poses(self, obj):
    # generate poses in the object frame
    loc_t = obj.GetTransform()
    return self._generate_poses(loc_t)

  def generate_pdp_poses(self, loc_t):
    # generate poses in the location frame
    return self._generate_poses(loc_t)

  def generate_curr_pose(self, obj_or_surface):
    # return robot's current pose in object/surface frame
    loc_t = obj_or_surface.GetTransform()
    pose = np.linalg.inv(loc_t).dot(self.robot.GetTransform())
    return (pose, loc_t)

  def _generate_poses(self, loc_t):
    if DISABLE_BASE:
      pose = np.linalg.inv(loc_t).dot(self.robot.GetTransform())
      yield (pose, loc_t)
      raise StopIteration

    pose_list = self._calc_pose_list()

    # BasePoseGenerator2.state.shuffle(pose_list)
    # sort by closest to current position
    def closest_distance(pose):
      cur_pose = self.robot.GetTransform()
      world_pose = self.get_world_frame_pose(pose, loc_t)
      world_pose_x = world_pose[0, 3]
      world_pose_y = world_pose[1, 3]
      cur_pose_x = cur_pose[0, 3]
      cur_pose_y = cur_pose[1, 3]
      return np.sqrt(pow(cur_pose_x - world_pose_x, 2) + pow(cur_pose_y - world_pose_y, 2))
    pose_list = sorted(pose_list, key=closest_distance)

    count = 0
    for pose in pose_list:
      if PRINT_GEN_COUNT:
        print("base pose gen2: " + repr(count))
        count += 1
      yield (pose, loc_t)
