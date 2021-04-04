import numpy as np
import openravepy
import utils


TARGET_FILE = '../environments/rll_tray_world.dae'
TABLE_HEIGHT = 0.657


def create_dest(env, destname, pos):
  dim = 0.1
  THICKNESS = 0.001

  surface = openravepy.KinBody.GeometryInfo()
  surface._type = openravepy.GeometryType.Box
  surface._vGeomData = [dim/2, dim/2, THICKNESS/2]
  surface._vDiffuseColor = [1, 0, 0]

  dest = openravepy.RaveCreateKinBody(env, '')
  dest.InitFromGeometries([surface])
  dest.SetName(destname)

  t = openravepy.matrixFromPose([1, 0, 0, 0] + list(pos))
  dest.SetTransform(t)

  return dest


def create_tray(env, t):
  dim1 = 0.3048
  dim2 = 0.6096
  THICKNESS = 0.0095
  TRAY_HEIGHT = 0.148

  surface = openravepy.KinBody.GeometryInfo()
  surface._type = openravepy.GeometryType.Box
  surface._vGeomData = [dim1/2, dim2/2, THICKNESS/2]
  surface._vDiffuseColor = [0.4, 0.2, 0.4]
  surface._t = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, TRAY_HEIGHT - THICKNESS/2))

  standoff = openravepy.KinBody.GeometryInfo()
  standoff._type = openravepy.GeometryType.Box
  standoff._vGeomData = [0.6*dim1/2, 0.6*dim2/2, TRAY_HEIGHT/2]
  standoff._vDiffuseColor = [0.4, 0.2, 0.4]
  standoff._t = openravepy.matrixFromPose((1, 0, 0, 0, 0, 0, TRAY_HEIGHT/2))

  tray = openravepy.RaveCreateKinBody(env, '')
  tray.InitFromGeometries([surface, standoff])
  tray.SetName('tray')

  pose = openravepy.poseFromMatrix(t)
  pose[:4] = [0.7071, 0, 0, 0.7071]
  t = openravepy.matrixFromPose(pose)
  tray.SetTransform(t)

  return tray


def create_cylinder(env, body_name, pos, radius, height):
  infocylinder = openravepy.KinBody.GeometryInfo()
  infocylinder._type = openravepy.GeometryType.Cylinder
  infocylinder._vGeomData = [radius, height]
  infocylinder._bVisible = True
  infocylinder._vDiffuseColor = [0, 1, 1]

  cylinder = openravepy.RaveCreateKinBody(env, '')
  cylinder.InitFromGeometries([infocylinder])
  cylinder.SetName(body_name)

  pos[2] += height/2
  cylinder.SetTransform(openravepy.matrixFromPose([1, 0, 0, 0] + pos))

  return cylinder


def create_table(env, body_name, dim1, dim2, pos):
  THICKNESS = 0.2

  LEGDIM1 = 1.3
  LEGDIM2 = 0.6
  LEGHEIGHT = 0.6  # Doesn't actually determine the height of the table in the env

  tabletop = openravepy.KinBody.GeometryInfo()
  tabletop._type = openravepy.GeometryType.Box
  tabletop._vGeomData = [dim1/2, dim2/2, THICKNESS/2]
  tabletop._t[2, 3] = -THICKNESS/2
  tabletop._vDiffuseColor = [0.5, 0.2, 0.1]

  leg1 = openravepy.KinBody.GeometryInfo()
  leg1._type = openravepy.GeometryType.Box
  leg1._vGeomData = [LEGDIM1/2, LEGDIM2/2, LEGHEIGHT/2]
  leg1._t[0, 3] = dim1/2 - LEGDIM1/2
  leg1._t[1, 3] = dim2/2 - LEGDIM2/2
  leg1._t[2, 3] = -LEGHEIGHT/2 - THICKNESS/2
  leg1._vDiffuseColor = [0.5, 0.2, 0.1]

  leg2 = openravepy.KinBody.GeometryInfo()
  leg2._type = openravepy.GeometryType.Box
  leg2._vGeomData = [LEGDIM1/2, LEGDIM2/2, LEGHEIGHT/2]
  leg2._t[0, 3] = dim1/2 - LEGDIM1/2
  leg2._t[1, 3] = -dim2/2 + LEGDIM2/2
  leg2._t[2, 3] = -LEGHEIGHT/2 - THICKNESS/2
  leg2._vDiffuseColor = [0.5, 0.2, 0.1]

  leg3 = openravepy.KinBody.GeometryInfo()
  leg3._type = openravepy.GeometryType.Box
  leg3._vGeomData = [LEGDIM1/2, LEGDIM2/2, LEGHEIGHT/2]
  leg3._t[0, 3] = -dim1/2 + LEGDIM1/2
  leg3._t[1, 3] = dim2/2 - LEGDIM2/2
  leg3._t[2, 3] = -LEGHEIGHT/2 - THICKNESS/2
  leg3._vDiffuseColor = [0.5, 0.2, 0.1]

  leg4 = openravepy.KinBody.GeometryInfo()
  leg4._type = openravepy.GeometryType.Box
  leg4._vGeomData = [LEGDIM1/2, LEGDIM2/2, LEGHEIGHT/2]
  leg4._t[0, 3] = -dim1/2 + LEGDIM1/2
  leg4._t[1, 3] = -dim2/2 + LEGDIM2/2
  leg4._t[2, 3] = -LEGHEIGHT/2 - THICKNESS/2
  leg4._vDiffuseColor = [0.5, 0.2, 0.1]

  table = openravepy.RaveCreateKinBody(env, '')
  table.InitFromGeometries([tabletop, leg1, leg2, leg3, leg4])
  table.SetName(body_name)

  table.SetTransform(openravepy.matrixFromPose([1, 0, 0, 0] + pos))

  return table


env = openravepy.Environment()
env.SetViewer('qtcoin')

# plot origin for sanity
t = np.eye(4)
o = utils.plot_transform(env, t, 1.0)

# spawn PR2
robot = env.ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml")
env.Add(robot)

# spawn main table
env.AddKinBody(create_table(env, 'table', 2.25, 0.94, [2.25, 0, TABLE_HEIGHT]))

# spawn tray table
env.AddKinBody(create_table(env, 'tray_table', 1.525, 0.61, [0.95, 1.0, TABLE_HEIGHT]))

# spawn glasses (object2)
GLASS_HEIGHT = 0.15
GLASS_RADIUS = 0.04
env.AddKinBody(create_cylinder(env, 'object21', [1.4, 0, TABLE_HEIGHT], GLASS_RADIUS, GLASS_HEIGHT))
env.AddKinBody(create_cylinder(env, 'object22', [1.4, 0.2, TABLE_HEIGHT], GLASS_RADIUS, GLASS_HEIGHT))
env.AddKinBody(create_cylinder(env, 'object23', [1.4, -0.2, TABLE_HEIGHT], GLASS_RADIUS, GLASS_HEIGHT))

# spawn bowls (object1)
BOWL_HEIGHT = 0.08
BOWL_RADIUS = 0.06
env.AddKinBody(create_cylinder(env, 'object11', [1.2, 0, TABLE_HEIGHT], BOWL_RADIUS, BOWL_HEIGHT))
env.AddKinBody(create_cylinder(env, 'object12', [1.2, 0.2, TABLE_HEIGHT], BOWL_RADIUS, BOWL_HEIGHT))
env.AddKinBody(create_cylinder(env, 'object13', [1.2, -0.2, TABLE_HEIGHT], BOWL_RADIUS, BOWL_HEIGHT))

# # spawn plates (object3)
# PLATE_HEIGHT = 0.04
# PLATE_RADIUS = 0.08
# env.AddKinBody(create_cylinder(env, 'object31', [1.4, -0.2, TABLE_HEIGHT], PLATE_RADIUS, PLATE_HEIGHT))
# env.AddKinBody(create_cylinder(env, 'object32', [1.2, -0.2, TABLE_HEIGHT], PLATE_RADIUS, PLATE_HEIGHT))

# spawn tray
env.AddKinBody(create_tray(env, openravepy.matrixFromPose((1, 0, 0, 0, 0.5, 0.85, TABLE_HEIGHT))))

# adding destinations
env.AddKinBody(create_dest(env, 'destobject11', (1.8, -0.3, TABLE_HEIGHT)))
env.AddKinBody(create_dest(env, 'destobject12', (2.2, -0.3, TABLE_HEIGHT)))
env.AddKinBody(create_dest(env, 'destobject13', (2.6, -0.3, TABLE_HEIGHT)))

env.AddKinBody(create_dest(env, 'destobject21', (2.0, -0.1, TABLE_HEIGHT)))
env.AddKinBody(create_dest(env, 'destobject22', (2.4, -0.1, TABLE_HEIGHT)))
env.AddKinBody(create_dest(env, 'destobject23', (2.8, -0.1, TABLE_HEIGHT)))

# env.AddKinBody(create_dest(env, 'destobject31', (2.6, -0.3, TABLE_HEIGHT)))
# env.AddKinBody(create_dest(env, 'destobject32', (2.8, -0.1, TABLE_HEIGHT)))

tl1 = create_dest(env, 'trayloc1', (0.5, 0.85, TABLE_HEIGHT))
env.AddKinBody(tl1)
tl2 = create_dest(env, 'trayloc2', (3.2, -0.4, TABLE_HEIGHT))
env.AddKinBody(tl2)


env.Save(TARGET_FILE)
raw_input("Press ENTER to exit!")
