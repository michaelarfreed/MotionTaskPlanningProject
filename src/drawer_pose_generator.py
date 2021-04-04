import openravepy
import numpy as np
import utils
from settings import seed
from env_manager import EnvManager


class DrawerPoseGenerator(object):
  state = np.random.RandomState(seed)

  def __init__(self, env, unmovable_objects, open):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.num_grasp_points = 3
    self.num_open_amounts = 10
    self.open = open
    self.height_offset = 0.0
    self.approach_dist = 0.05
    self.retreat_dist = 0.05

  def get_world_frame_pose(self, pose):
    return self.grabpoint_t.dot(pose)

  def generate_poses(self, obj):
    # generate poses in the object frame
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)

      obj.SetTransform(np.identity(4))  # need to compute object info at origin
      min_x, max_x, min_y, max_y, z = utils.get_object_limits(obj)

      EnvManager.restore_openrave_state(self.env, saved_env)

    obj_t = obj.GetTransform()
    grabpoint_x = obj_t[0, 3] + min_x
    grabpoint_y = obj_t[1, 3] 
    grabpoint_z = obj_t[2, 3] + z
    self.grabpoint_t = openravepy.matrixFromPose((1, 0, 0, 0,
                                                  grabpoint_x, grabpoint_y, grabpoint_z))

    pose_list = []
    grab_space = np.linspace(min_y + 0.02, max_y - 0.02, num=self.num_grasp_points)
    if self.open:
      move_dist_range = np.linspace(0.2, 0.4, num=self.num_open_amounts)
    else:
      move_dist_range = [-EnvManager.drawer_open_dist] # can't be in init method

    for i in range(self.num_grasp_points):
      for move_dist in move_dist_range:
        r0 = openravepy.matrixFromQuat((0, 0, 0, 1))
        r1 = openravepy.matrixFromAxisAngle((0, np.pi/2, 0))
        r2 = openravepy.matrixFromAxisAngle((0, 0, -np.pi/2))
        t1 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, self.height_offset))
        t2 = openravepy.matrixFromPose((1, 0, 0, 0, 0, grab_space[i], 0))
        open_pose_start = t2.dot(t1).dot(r1).dot(r2).dot(r0)

        t3 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -self.approach_dist))
        pre_pose = open_pose_start.dot(t3)

        t4 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -move_dist))
        open_pose_end = open_pose_start.dot(t4)

        t5 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -self.retreat_dist))
        retreat_pose = open_pose_end.dot(t5)

        pose_list.append((pre_pose, open_pose_start, open_pose_end, retreat_pose))

    DrawerPoseGenerator.state.shuffle(pose_list)
    for pose in pose_list:
      yield pose
