import openravepy
import numpy as np
import utils
from env_manager import EnvManager

class TrayPickupPoseGenerator(object):
  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.approach_dist = 0.15
    self.dist_offset = 0.03
    self.lift_dist = 0.2
    self.grip_width = 0.8  # in percent

  @classmethod
  def get_world_frame_pose(self, pose, loc_t):
    return loc_t.dot(pose)

  def generate_poses(self, tray):
    # generate poses in the tray frame
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      tray.SetTransform(np.identity(4))  # need to compute tray dimensions at origin
      _, _, min_y, max_y, tray_z = utils.get_object_limits(tray)
      tray_width = max_y - min_y
      EnvManager.restore_openrave_state(self.env, saved_env)

    r1 = openravepy.matrixFromAxisAngle((0, np.pi/2, 0))
    r2 = openravepy.matrixFromAxisAngle((0, 0, np.pi/2))
    r3 = openravepy.matrixFromAxisAngle((0, np.pi/2, 0))
    r_grasp_pose = r1.dot(r2).dot(r3)
    r_grasp_pose[1][3] -= tray_width/2 - self.dist_offset
    r_grasp_pose[2][3] += tray_z

    t4 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -self.approach_dist))
    t5 = openravepy.matrixFromPose((1, 0, 0, 0, 0, self.lift_dist, 0))

    r_pre_grasp_pose = r_grasp_pose.dot(t4)
    r_lift_pose = r_grasp_pose.dot(t5)

    # left
    rot = openravepy.matrixFromAxisAngle((0, 0, np.pi))
    l_pre_grasp_pose = rot.dot(r_pre_grasp_pose)
    l_grasp_pose = rot.dot(r_grasp_pose)
    l_lift_pose = rot.dot(r_lift_pose)

    yield (r_pre_grasp_pose, r_grasp_pose, r_lift_pose,
           l_pre_grasp_pose, l_grasp_pose, l_lift_pose)
