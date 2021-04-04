import trajoptpy.math_utils as mu
import numpy as np
import utils
import openravepy
from env_manager import EnvManager


class CollisionChecker(object):
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]

  def _get_robot_collisions(self, include_obj_world_col):
    """
    Returns a set with all objects the robot and in-hand-obects
    are colliding with
    """
    collisions = set()
    report = openravepy.CollisionReport()

    for link in self.robot.GetLinks():
      if self.env.CheckCollision(link, report=report):
        collisions.add(report.plink2.GetParent())

    #collisions = collisions.union(self.add_all_collisions(obj, report, grabbed_objects))
    if include_obj_world_col:
      grabbed_objects = EnvManager.grabbed['rightarm'].union(EnvManager.grabbed['leftarm'])
      for obj_name in grabbed_objects:
        obj = self.env.GetKinBody(obj_name)
        if obj is None:
          continue
        self.robot.Release(obj)
        for obj_link in obj.GetLinks():
          if self.env.CheckCollision(obj_link, report=report):
            col1 = report.plink2.GetParent()
            col_name = col1.GetName()
            if col_name not in grabbed_objects and 'loc' not in col_name and 'dest' not in col_name\
                  and not col1.IsRobot():
              collisions.add(col1)
        if obj_name in EnvManager.grabbed['rightarm']:
          manip = 'rightarm'
        elif obj_name in EnvManager.grabbed['leftarm']:
          manip = 'leftarm'
        manip_orig = self.robot.GetActiveManipulator().GetName()

        self.robot.SetActiveManipulator(manip)
        self.robot.Grab(obj)
        # self.robot.SetActiveManipulator(manip_orig)

        
      
      # Proper detection of in-hand object-world collision.
      # if include_obj_world_col:
      #   if col_name in grabbed_objects:
      #     collisions.add(report.plink2.GetParent())
    return collisions

  def get_collisions(self, include_obj_world_col=True):
    return self._get_robot_collisions(include_obj_world_col)

  def get_traj_collisions(self, traj, n=100, include_obj_world_col=True):
    collisions = set()
    traj_up = mu.interp2d(
      np.linspace(0, 1, n), np.linspace(0, 1, len(traj)), traj)
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)
      for dof_values in traj_up:
        utils.update_robot(self.robot, dof_values)
        for obj in self._get_robot_collisions(include_obj_world_col):
          collisions.add(obj)
      EnvManager.restore_openrave_state(self.env, saved_env)
    return collisions

  def add_all_collisions(self, obj, report, grabbed):
    cols = set()
    removed = []

    while self.env.CheckCollision(obj, report=report):
      col1 = report.plink1.GetParent()
      col_name = col1.GetName()
      if col_name not in grabbed and 'loc' not in col_name and 'dest' not in col_name:
        cols.add(col1)
      removed.append(col1)
      self.env.Remove(col1)

    for removed_obj in removed:
      self.env.AddKinBody(removed_obj)

    return cols
