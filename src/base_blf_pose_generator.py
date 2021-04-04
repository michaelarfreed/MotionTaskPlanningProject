import numpy as np
import openravepy
import utils
from collision_checker import CollisionChecker
from settings import seed, DISABLE_BASE


class BaseBLFPoseGenerator(object):
  cached_poses = {}
  cached_pose_lists = {}

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.body_offset = np.linspace(0.2, 0.5, num=3)  # distance offset from edge of body
    self.sampling_dist = 0.5  # poses are separated by at most this amount for a given body_offset
    self.collision_checker = CollisionChecker(self.env)

  def _calc_pose_list(self, body):
    # generate poses in the body frame
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)

      # do all computation with body at origin
      # so AABBs actually line up to rectangular bodies
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
          pose = openravepy.matrixFromPose(quat + [x, y, 0])
          pose_list.append(pose)

      EnvManager.restore_openrave_state(self.env, saved_env)
      return pose_list

  def generate_poses(self, body, error_free_only=True):
    """
    Generates base poses around a given body
    """
    if DISABLE_BASE:
      yield self.robot.GetTransform(), True
      raise StopIteration

    saved_env = EnvManager.save_openrave_state(self.env)
    pose_list_key = body.GetName()
    pose_key = body.GetName()
    body_t = body.GetTransform()

    pose_list = self.cached_pose_lists.get(
      pose_list_key, self._calc_pose_list(body))
    state = np.random.RandomState(seed)
    state.shuffle(pose_list)
    if pose_key in self.cached_poses:
      pose_list = [self.cached_poses[pose_key]] + pose_list

    # check collisions and yield poses
    for pose in pose_list:
      base_t = body_t.dot(pose)
      base_t[2][3] = 0  # make sure base is on the ground
      with self.env:
        self.robot.SetTransform(pose)
        collisions = self.collision_checker.get_collisions()
        collision_free = collisions == set()
        if (error_free_only and not collision_free) or\
           (collisions.intersection(self.unmovable_objects) != set()):
          EnvManager.restore_openrave_state(self.env, saved_env)
          continue

        EnvManager.restore_openrave_state(self.env, saved_env)
      self.cached_poses[pose_key] = pose
      yield base_t, collision_free

    with self.env:
      EnvManager.restore_openrave_state(self.env, saved_env)
