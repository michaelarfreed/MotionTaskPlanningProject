import openravepy
import numpy as np
import utils
from settings import seed
from env_manager import EnvManager

class LipPickupPoseGenerator(object):
  """
  works for rotationally symmetric objects with a lip, like glasses and bowls
  """
  state = np.random.RandomState(seed)

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.num_grasps = 8
    self.dist_offset = 0
    self.approach_dist = 0.05
    self.approach_ang = 1.35  # radians from horizontal
    self.lift_dist = 0.05

  @classmethod
  def get_world_frame_pose(self, pose, loc_t):
    return loc_t.dot(pose)

  def generate_poses(self, obj):
    # generate poses in the object frame
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)

      obj.SetTransform(np.identity(4))  # need to compute object info at origin
      min_x, max_x, min_y, max_y, z = utils.get_object_limits(obj)
      obj_radius = (max_x - min_x + max_y - min_y) / 4

      EnvManager.restore_openrave_state(self.env, saved_env)

    pose_list = []
    for i in range(self.num_grasps):
      rot_ang = i * (2 * np.pi) / self.num_grasps

      r0 = openravepy.matrixFromAxisAngle((np.pi/2, 0, 0))
      r1 = openravepy.matrixFromAxisAngle((0, np.pi/2, 0))
      r2 = openravepy.matrixFromAxisAngle((0, self.approach_ang, 0))
      t1 = openravepy.matrixFromPose((1, 0, 0, 0, -obj_radius, 0, z - self.dist_offset))
      r3 = openravepy.matrixFromAxisAngle((0, 0, rot_ang))
      grasp_pose = r3.dot(t1).dot(r2).dot(r0).dot(r1)

      t4 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -self.approach_dist))
      pre_grasp_pose = grasp_pose.dot(t4)

      lift_pose = grasp_pose.copy()
      lift_pose[2, 3] += self.lift_dist

      pose_list.append((pre_grasp_pose, grasp_pose, lift_pose))

    LipPickupPoseGenerator.state.shuffle(pose_list)
    for pose in pose_list:
      yield pose
