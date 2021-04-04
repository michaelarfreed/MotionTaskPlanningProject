import copy
import trajoptpy
import openravepy


class EnvManager(object):
  init_bodies = []
  tray_stack = []
  tray_stack_height = 0.0
  grabbed = {'rightarm': set(),
             'leftarm': set()}
  object_groups = {}  # key: base body name, values: set of body names (Ex: {'tray': {'object11', object21}})
  init_groups = {}
  drawer_init_x = None
  drawer_open_dist = 0
  non_robot_links = set()

  @classmethod
  def init_openrave_state(self, env):
    EnvManager.init_bodies = env.GetBodies()

    for body in EnvManager.init_bodies:
      body_name = body.GetName()
      if not body.IsRobot():
        for link in body.GetLinks():
          EnvManager.non_robot_links.add(link)

  @classmethod
  def save_openrave_state(self, env):
    data = {'Robots': [],
            'EnvBodies': [],
            'AllBodies': [],
            'EnvManagerState': EnvManager.save_state()}
    for body in env.GetBodies():
      body_name = body.GetName()
      if body.IsRobot():
        robot = env.GetRobot(body_name)
        data['Robots'].append((body_name, EnvManager.save_robot_state(robot)))
      else:
        data['EnvBodies'].append((body, body.GetTransform()))
    for body in EnvManager.init_bodies:
      data['AllBodies'].append((body, body.GetTransform()))
    return data

  @classmethod
  def restore_openrave_state(self, env, data):
    for robot_name, values in data['Robots']:
      robot = env.GetRobot(robot_name)
      EnvManager.restore_robot_state(robot, values)

    for body, body_t in data['EnvBodies']:
      if body not in env.GetBodies():
        env.AddKinBody(body)

    for body, body_t in data['AllBodies']:
      body.SetTransform(body_t)

    # EnvManager._reset_collisions(env)

    for robot_name, values in data['Robots']:
      robot = env.GetRobot(robot_name)
      EnvManager.restore_robot_grabbed_objects(robot, values)

    EnvManager.restore_state(data['EnvManagerState'])

    # if EnvManager.tray_stack != []:
    #   tray_bodies = [env.GetKinBody(body_name) for body_name in EnvManager.tray_stack + ['tray']]
    #   self.exclude_all_collisions(env, tray_bodies)

  @classmethod
  def save_state(self):
    # make copies of everything
    return {'tray_stack': list(EnvManager.tray_stack),
            'tray_stack_height': EnvManager.tray_stack_height,
            'grabbed': copy.deepcopy(EnvManager.grabbed)}

  @classmethod
  def restore_state(self, state):
    # make copies of everything
    EnvManager.tray_stack = list(state['tray_stack'])
    EnvManager.tray_stack_height = state['tray_stack_height']
    EnvManager.grabbed = copy.deepcopy(state['grabbed'])

  @classmethod
  def save_robot_state(self, robot):
    # make copies of everything
    values = {}
    values['Transform'] = copy.deepcopy(robot.GetTransform())
    values['DOFValues'] = copy.deepcopy(robot.GetDOFValues())
    values['Grabbed'] = copy.deepcopy(EnvManager.grabbed)
    return values

  @classmethod
  def restore_robot_state(self, robot, values):
    # make copies of everything
    robot.SetTransform(copy.deepcopy(values['Transform']))
    robot.SetDOFValues(copy.deepcopy(values['DOFValues']))

  @classmethod
  def restore_robot_grabbed_objects(self, robot, values):
    for obj in robot.GetGrabbed():
      robot.Release(obj)
      EnvManager.include_robot_grabbed_collisions(robot, obj)
    EnvManager.grab_all(robot, values['Grabbed'])

  @classmethod
  def include_robot_grabbed_collisions(self, robot, grabbed):
    env = robot.GetEnv()
    cc = trajoptpy.GetCollisionChecker(env)
    for robotlink in robot.GetLinks():
      for grabbedlink in grabbed.GetLinks():
        cc.IncludeCollisionPair(robotlink, grabbedlink)
        cc.IncludeCollisionPair(grabbedlink, grabbedlink)

  @classmethod
  def exclude_manip_grabbed_collisions(self, robot, grabbed, manip):
    manip_to_use = robot.GetManipulator(manip)
    env = robot.GetEnv()
    cc = trajoptpy.GetCollisionChecker(env)
    for maniplink in manip_to_use.GetChildLinks():
      if 'gripper' in maniplink.GetName():
        for grabbedlink in grabbed.GetLinks():
          cc.ExcludeCollisionPair(maniplink, grabbedlink)
          cc.ExcludeCollisionPair(grabbedlink, grabbedlink)

  @classmethod
  def _reset_collisions(self, env):
    cc = trajoptpy.GetCollisionChecker(env)
    env_links = set()
    for b in env.GetBodies():
      if b.GetEnvironmentId() == 0:
        print "Warning: Had to re-add object {}".format(b.GetName())
        env.AddKinBody(b)
      for l in b.GetLinks():
        env_links.add(l)

    for link1 in EnvManager.non_robot_links:
      if link1 not in env_links:
        continue
      for link2 in EnvManager.non_robot_links:
        if link2 not in env_links:
          continue
        cc.IncludeCollisionPair(link1, link2)

  @classmethod
  def grab_all(self, robot, grabbed):
    active_manip = robot.GetActiveManipulator()
    for manip, objects in grabbed.items():
      robot.SetActiveManipulator(manip)
      for obj_name in objects:
        obj = robot.GetEnv().GetKinBody(obj_name)
        if obj is not None:
          robot.Grab(obj)
          EnvManager.exclude_manip_grabbed_collisions(robot, obj, manip)
    robot.SetActiveManipulator(active_manip)

  @classmethod
  def setup_drawer_object_groups(self, env):
    bodies = env.GetBodies()
    for body in bodies:
      body_name = body.GetName()
      if 'drawer' in body_name and 'outer' not in body_name:
        min_x, max_x, min_y, max_y, min_z, max_z = EnvManager.get_object_limits_2(body)
        inside_set = set()
        init_groups_set = set()
        for sub_body in bodies:
          sub_body_name = sub_body.GetName()

          # prevent drawer and drawer outer from being added to the set
          if body is not sub_body and 'drawer' not in sub_body_name:
            sub_body_t = sub_body.GetTransform()
            sub_body_x = sub_body_t[0, 3]
            sub_body_y = sub_body_t[1, 3]
            sub_body_z = sub_body_t[2, 3]

            if ((min_x < sub_body_x < max_x) and
               (min_y < sub_body_y < max_y) and
               (min_z < sub_body_z < max_z)):
              inside_set.add(sub_body_name)
              init_groups_set.add(sub_body)
        EnvManager.object_groups[body_name] = inside_set
        EnvManager.init_groups[body_name] = init_groups_set

  @classmethod
  def update_drawer_object_groups(self, env):
    for body_name in EnvManager.init_groups.keys():
      if 'drawer' in body_name:
        min_x, max_x, min_y, max_y, min_z, max_z = EnvManager.get_object_limits_2(env.GetKinBody(body_name))
        new_set = set()
        for sub_body in EnvManager.init_groups[body_name]:
          sub_body_t = sub_body.GetTransform()
          sub_body_x = sub_body_t[0, 3]
          sub_body_y = sub_body_t[1, 3]
          sub_body_z = sub_body_t[2, 3]

          if ((min_x < sub_body_x < max_x) and
              (min_y < sub_body_y < max_y) and
              (min_z < sub_body_z < max_z)):
            new_set.add(sub_body.GetName())
        EnvManager.object_groups[body_name] = new_set

  @classmethod
  def get_object_limits_2(self, obj):
    """
    Returns the bounding box of an object.
    Returns: min_x, max_x, min_y, max_y, min_z, max_z
    """

    ab = obj.ComputeAABB()
    max_x = ab.pos()[0] + ab.extents()[0]
    min_x = ab.pos()[0] - ab.extents()[0]

    max_y = ab.pos()[1] + ab.extents()[1]
    min_y = ab.pos()[1] - ab.extents()[1]

    max_z = ab.pos()[2] + ab.extents()[2]
    min_z = ab.pos()[2] - ab.extents()[2]

    return min_x, max_x, min_y, max_y, min_z, max_z

  @classmethod
  def exclude_all_collisions(self, env, objects):
    cc = trajoptpy.GetCollisionChecker(env)
    for obj1 in objects:
      if obj1 is None:
        continue
      for obj2 in objects:
        if obj2 is None:
          continue
        for obj1link in obj1.GetLinks():
          for obj2link in obj2.GetLinks():
            cc.ExcludeCollisionPair(obj1link, obj2link)

if __name__ == "__main__":
  import openravepy
  import numpy as np
  env = openravepy.Environment()
  env.Load("../environments/created_info.dae")
  EnvManager.init_openrave_state(env)

  
  obj = env.GetKinBody("object11")
  T = obj.GetTransform()
  T[2, 3] += 10
  obj.SetTransform(T)
  env.Remove(obj)
  saved = EnvManager.save_openrave_state(env)
  obj.SetTransform(np.eye(4))
  EnvManager.restore_openrave_state(env, saved)
  print(env.GetBodies())
  print(obj.GetTransform())
