import openravepy
import numpy as np
import utils
from collision_checker import CollisionChecker
from settings import seed, DISABLE_BASE


class BaseGPPoseGenerator(object):
  cached_poses = {}
  cached_pose_lists = {}

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.num_poses_per_ring = 8
    self.dist_offset_range = np.linspace(0.5, 0.85, num=2)
    self.angle_range = [0]  # np.linspace(-np.pi/8, np.pi/8, num=3)
    self.collision_checker = CollisionChecker(self.env)

  def _calc_pose_list(self):
    # generate poses in the object frame
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

  def generate_poses(self, obj, manip, error_free_only=True):
    if DISABLE_BASE:
      yield self.robot.GetTransform(), True
      raise StopIteration

    saved_env = EnvManager.save_openrave_state(self.env)
    pose_list_key = ''
    pose_key = '{}_{}'.format(obj.GetName(), manip)
    obj_t = obj.GetTransform()

    pose_list = self.cached_pose_lists.get(
      pose_list_key, self._calc_pose_list())
    state = np.random.RandomState(seed)
    state.shuffle(pose_list)
    if pose_key in self.cached_poses:
      pose_list = [self.cached_poses[pose_key]] + pose_list

    # check collisions and yield poses
    for pose in pose_list:
      base_t = obj_t.dot(pose)
      base_t[2][3] = 0  # make sure base is on the ground
      with self.env:
        self.robot.SetTransform(base_t)
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
