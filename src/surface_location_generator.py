import openravepy
import numpy as np
import utils
from settings import seed
import pdb
import time
from env_manager import EnvManager

class SurfaceLocationGenerator(object):
  state = np.random.RandomState(seed)
  location_lists = {}

  def __init__(self, env, unmovable_objects):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.location_diff = 0.08  # locations will be approx this far apart
    self.edge_padding = 0.04  # distance from edge of surface not to put down
    self.reachable_dist = 0.8  # distance from base_footprint that is reachable

  def _calc_location_list(self, surface, random = True, num = 40):
    # generate locations in the surface frame
    with self.env:
      saved_env = EnvManager.save_openrave_state(self.env)

      # do all computation with surface at origin
      # so AABBs actually line up to rectangular surfaces
      surface.SetTransform(np.identity(4))

      min_x, max_x, min_y, max_y, z = utils.get_object_limits(surface)
      min_x += self.edge_padding
      max_x -= self.edge_padding
      min_y += self.edge_padding
      max_y -= self.edge_padding
      # print(min_x, max_x)
      # a = utils.plot_transform(self.env, surface.GetTransform())
      # raw_input("plotted")

      EnvManager.restore_openrave_state(self.env, saved_env)

    x_diff = max_x - min_x
    y_diff = max_y - min_y
    num_x = int(x_diff / self.location_diff)
    num_y = int(y_diff / self.location_diff)
    location_list = []

    if random:
      x_list = map(lambda x: min_x + x*x_diff, \
                     SurfaceLocationGenerator.state.random_sample(num))
      y_list = map(lambda y: min_y + y*y_diff, \
                     SurfaceLocationGenerator.state.random_sample(num))
    else:
      x_list = np.linspace(min_x, max_x, num=num)
      y_list = np.linspace(min_y, max_y, num=num)

    coord_list = zip(x_list, y_list)
    for x, y in coord_list:
      loc = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
      location_list.append(loc)

    SurfaceLocationGenerator.location_lists[surface.GetName()] = location_list

  def _generate_tray_loc(self, tray):
    loc_t = tray.GetTransform()
    tray_height = utils.get_object_height(tray)
    loc_t[2][3] += tray_height
    loc_t[2][3] += EnvManager.tray_stack_height
    return loc_t

  def generate_locations(self, surface, collision_free=True, reachable_only=True):
    if surface.GetName() == 'tray':
      yield self._generate_tray_loc(surface)
      raise StopIteration
    elif 'loc' in surface.GetName() or 'dest' in surface.GetName() or 'temp' in surface.GetName():
      yield surface.GetTransform()
      raise StopIteration

    surface_t = surface.GetTransform()

    # use precomputed locations if possible
    # if surface.GetName() not in SurfaceLocationGenerator.location_lists:
    self._calc_location_list(surface)
    location_list = SurfaceLocationGenerator.location_lists[surface.GetName()]

    if reachable_only:
      base_footprint_t = self.robot.GetTransform()
      base_x = base_footprint_t[0, 3]
      base_y = base_footprint_t[1, 3]

      def reachable(location):
        location_x = location[0, 3]
        location_y = location[1, 3]
        dist = np.sqrt(pow(base_x - location_x, 2) + pow(base_y - location_y, 2))
        return dist <= self.reachable_dist

      location_list = filter(reachable, location_list)

    SurfaceLocationGenerator.state.shuffle(location_list)

    i = 0
    for loc in location_list:
      if collision_free:
        i+=1
        body = utils.create_body_at(self.env, surface_t.dot(loc))
        self.env.Add(body)
        collisions = utils.get_all_collisions(body, self.env)
        # print "dummy added: "+ repr(i)
        # time.sleep(0.05)
        self.env.Remove(body)
        if len(collisions)>0:
          continue

      yield surface_t.dot(loc)
