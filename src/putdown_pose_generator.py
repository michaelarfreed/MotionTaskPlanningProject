import openravepy
import numpy as np
import utils
from settings import seed, PRINT_GEN_COUNT
from env_manager import EnvManager
from surface_location_generator import SurfaceLocationGenerator
import pdb


class PutdownPoseGenerator(object):
  state = np.random.RandomState(seed)

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.poses_per_location = 8
    self.lowering_height = 0.05
    self.release_height = 0.00  # from location origin to bottom of object
    self.retreat_dist = 0.15
    self.surface_location_generator = SurfaceLocationGenerator(env, 
                                                  unmovable_objects)

  @classmethod
  def get_world_frame_pose(self, pose, loc_t):
    return loc_t.dot(pose)

  def generate_pdp_for_surface(self, surface, obj, manip):
    location_list = self.surface_location_generator.generate_locations(surface)

    count = 0
    for loc_t in location_list:
      pose_list = self.generate_poses(obj, manip)
      if PRINT_GEN_COUNT:
        print("putdown gen: " + repr(count))
        count += 1
      for pose in pose_list:
        yield (loc_t, pose)

  def generate_poses(self, obj, manip):
    # generate poses in the location frame
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      obj.SetTransform(np.identity(4))  # need to compute object height at origin
      obj_height = utils.get_object_height(obj)
      EnvManager.restore_openrave_state(self.env, saved_env)

      obj_t = obj.GetTransform()
      if manip == 'rightarm':
        link = 'r_gripper_tool_frame'
      elif manip == 'leftarm':
        link = 'l_gripper_tool_frame'
      link_t = self.robot.GetLink(link).GetTransform()

      # find obj/gripper offset
      # solving: link_t = obj_t * A
      # A = inv(obj_t) * link_t
      A = np.linalg.inv(obj_t).dot(link_t)
      A[2, 3] += self.release_height + obj_height/2

    pose_list = []
    for i in range(self.poses_per_location):
      rot_ang = i * (2 * np.pi) / self.poses_per_location

      r1 = openravepy.matrixFromAxisAngle((0, np.pi/2, 0))
      r2 = openravepy.matrixFromAxisAngle((0, 0, rot_ang))
      pose = r2.dot(A).dot(r1)

      pre_pose = pose.copy()
      pre_pose[2, 3] += self.lowering_height

      t4 = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, -self.retreat_dist))
      post_pose = pose.dot(t4)

      pose_list.append((pre_pose, pose, post_pose))

    PutdownPoseGenerator.state.shuffle(pose_list)
    for pose in pose_list:
      yield pose
