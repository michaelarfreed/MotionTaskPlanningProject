import numpy as np
import openravepy
import utils
import time
from env_manager import EnvManager
from settings import seed, PRINT_GEN_COUNT

class BaseAroundPoseGenerator(object):
  state = np.random.RandomState(seed)
  
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.body_offset = np.linspace(0.2, 0.5, num=2)  # distance offset from edge of body
    self.sampling_dist = 0.75  # poses are separated by at most this amount for a given body_offset

  @classmethod
  def get_world_frame_pose(self, base_pose, body_t):
    base_t = body_t.dot(base_pose)
    base_t[2][3] = 0  # make sure base is on the ground
    return base_t

  def generate_curr_pose(self, body):
    # return robot's current pose in body frame
    body_t = body.GetTransform()
    pose = np.linalg.inv(body_t).dot(self.robot.GetTransform())
    return pose

  def generate_poses(self, body):
    """
    Generates base poses around a given body in the body's frame
    """
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      # do all computation with body at origin
      # so AABBs actually line up to rectangular bodies
      body_t = body.GetTransform()
      body.SetTransform(np.identity(4))

      min_x, max_x, min_y, max_y, _ = utils.get_object_limits(body)

      pose_list = []
      for offset in self.body_offset:
        min_x2 = min_x - offset
        max_x2 = max_x + offset
        min_y2 = min_y - offset
        max_y2 = max_y + offset

        num_x = np.ceil((max_x2 - min_x2) / self.sampling_dist)
        num_y = np.ceil((max_y2 - min_y2) / self.sampling_dist)
        x_range = np.linspace(min_x2, max_x2, num=num_x)
        y_range = np.linspace(min_y2, max_y2, num=num_y)

        coordinates = set()
        for x in x_range:
          coordinates.add((x, y_range[0]))
          coordinates.add((x, y_range[-1]))
        for y in y_range:
          coordinates.add((x_range[0], y))
          coordinates.add((x_range[-1], y))

        for x, y in coordinates:
          # TODO: figure out a better way to do this...
          if x >= max_x and y >= max_y:
            rot = -3*np.pi/4
          elif x >= max_x and y <= min_y:
            rot = 3*np.pi/4
          elif x <= min_x and y >= max_y:
            rot = -np.pi/4
          elif x <= min_x and y <= min_y:
            rot = np.pi/4
          elif x >= max_x:
            rot = np.pi
          elif x <= min_x:
            rot = 0
          elif y >= max_y:
            rot = -np.pi/2
          elif y <= min_y:
            rot = np.pi/2
          else:
            continue

          quat = openravepy.quatFromAxisAngle((0, 0, rot)).tolist()
          base_t = openravepy.matrixFromPose(quat + [x, y, 0])
          pose_list.append(base_t)

      EnvManager.restore_openrave_state(self.env, saved_env)

    def closest_distance(pose):
      cur_pose = self.robot.GetTransform()
      world_pose = self.get_world_frame_pose(pose, body_t)
      world_pose_x = world_pose[0, 3]
      world_pose_y = world_pose[1, 3]
      cur_pose_x = cur_pose[0, 3]
      cur_pose_y = cur_pose[1, 3]
      return np.sqrt(pow(cur_pose_x - world_pose_x, 2) + pow(cur_pose_y - world_pose_y, 2))
    pose_list = sorted(pose_list, key=closest_distance)

    count = 0
    for pose in pose_list:
      if PRINT_GEN_COUNT:
        print("base around pose gen: " + repr(count))
        count += 1
      yield pose
