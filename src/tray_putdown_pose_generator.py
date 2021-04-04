import openravepy
import numpy as np
import utils
from settings import seed


class TrayPutdownPoseGenerator(object):
  state = np.random.RandomState(seed)

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.poses_per_location = 4
    self.lowering_height = 0.1
    self.release_height = 0.00  # from location origin to bottom of tray
    self.retreat_dist = 0.15

  @classmethod
  def get_world_frame_pose(self, pose, loc_t):
    return loc_t.dot(pose)

  def generate_poses(self, tray):
    # generate poses in the location frame
    with self.env:
      tray_t = tray.GetTransform()
      r_link_t = self.robot.GetLink('r_gripper_tool_frame').GetTransform()
      l_link_t = self.robot.GetLink('l_gripper_tool_frame').GetTransform()

      # find tray/gripper offset
      # solving: link_t = tray_t * A
      # A = inv(tray_t) * link_t
      r_A = np.linalg.inv(tray_t).dot(r_link_t)
      r_A[2, 3] += self.release_height
      l_A = np.linalg.inv(tray_t).dot(l_link_t)
      l_A[2, 3] += self.release_height

    pose_list = []
    for i in range(self.poses_per_location):
      rot_ang = i * (2 * np.pi) / self.poses_per_location

      r1 = openravepy.matrixFromAxisAngle((0, np.pi/2, 0))
      r2 = openravepy.matrixFromAxisAngle((0, 0, rot_ang))
      r_pose = r2.dot(r_A).dot(r1)
      l_pose = r2.dot(l_A).dot(r1)

      r_pre_pose = r_pose.copy()
      r_pre_pose[2, 3] += self.lowering_height
      l_pre_pose = l_pose.copy()
      l_pre_pose[2, 3] += self.lowering_height

      t4 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -self.retreat_dist))
      r_post_pose = r_pose.dot(t4)
      l_post_pose = l_pose.dot(t4)

      pose_list.append((r_pre_pose, r_pose, r_post_pose,
                        l_pre_pose, l_pose, l_post_pose))

    # TrayPutdownPoseGenerator.state.shuffle(pose_list)
    for pose in pose_list:
      yield pose
