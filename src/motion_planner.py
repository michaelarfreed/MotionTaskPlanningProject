import json
import trajoptpy
import openravepy
import numpy as np
import utils
from rapprentice import math_utils as mu
from collision_checker import CollisionChecker
from settings import seed


class MotionPlanner(object):
  state = np.random.RandomState(seed)

  def __init__(self, env, n_steps=30):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.collision_checker = CollisionChecker(self.env)
    self.n_steps = n_steps

  def plan_with_pose(self, pos, rot,
                     collisionfree=True,
                     dof_targets=None,
                     n_steps=None):
    raise NotImplementedError("generate_traj_with_pose() not implemented!")

  def plan_with_dofs(self, dof_targets,
                     collisionfree=True,
                     n_steps=None):
    raise NotImplementedError("generate_traj_with_dofs() not implemented!")

  def optimize_traj(self, traj, manip):
    raise NotImplementedError("optimize_traj() not implemented!")


class TrajoptPlanner(MotionPlanner):
  def __init__(self, env, n_steps=30):
    super(TrajoptPlanner, self).__init__(env, n_steps)
    # self.viewer = trajoptpy.GetViewer(self.env)
    # trajoptpy.SetInteractive(True)

  def _generate_traj(self, costs, constraints, n_steps, collisionfree,
                     dof_targets, init_data, maintain_rel, maintain_up):
    with self.env:
      if type(dof_targets) == np.ndarray:
        dof_targets = dof_targets.tolist()

      if init_data is not None:
        if type(init_data) == np.ndarray:
          init_data = init_data.tolist()
        # getting proper start and end
        init_data[0] = self.robot.GetActiveDOFValues().tolist()
        if dof_targets is not None:
          init_data[-1] = dof_targets
        init_info = {
          "type": "given_traj",
          "data": init_data
        }
      elif dof_targets is not None:
        init_info = {
          "type": "straight_line",  # straight line in joint space.
          "endpoint": dof_targets
        }
      else:
        init_info = {
          "type": "stationary"
        }

      request = {
        "basic_info": {
          "n_steps": self.n_steps if (n_steps is None) else n_steps,
          "manip": "active",
          "start_fixed": True  # i.e., DOF values at first timestep are fixed based on current robot state
        },
        "costs": costs,
        "constraints": constraints,
        "init_info": init_info
      }

      request['costs'] += [{
        "type": "collision",
        "name": "cont_col_free",
        "params": {
          "coeffs": [50],
          "dist_pen": [0.05]
        }
      }, {
        "type": "collision",
        "name": "col",
        "params": {
          "continuous": False,
          "coeffs": [20],
          "dist_pen": [0.02]
        }
      }]

      prob = trajoptpy.ConstructProblem(json.dumps(request), self.env)

      if maintain_rel:  # maintain relative gripper poses
        r_link_t = self.robot.GetLink("r_gripper_tool_frame").GetTransform()
        l_link_t = self.robot.GetLink("l_gripper_tool_frame").GetTransform()

        # solving r_link_t = l_link_t * A
        # A = inv(l_link_t) * r_link_t
        A = np.linalg.inv(l_link_t).dot(r_link_t)

        def rel(x):
          self.robot.SetDOFValues(
            x,
            np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
                  self.robot.GetManipulator("leftarm").GetArmIndices()],
            False)

          r_t = self.robot.GetLink("r_gripper_tool_frame").GetTransform()
          l_t = self.robot.GetLink("l_gripper_tool_frame").GetTransform()

          A2 = np.linalg.inv(l_t).dot(r_t)
          return openravepy.poseFromMatrix(A.dot(np.linalg.inv(A2)))[1:]

        for t in xrange(1, n_steps):
          prob.AddConstraint(rel, [(t, j) for j in xrange(14)], "EQ", "rel%i" % t)

      if maintain_up:  # grippers can only be rotated about z-axis
        r_link_t = self.robot.GetLink("r_gripper_tool_frame").GetTransform()
        l_link_t = self.robot.GetLink("l_gripper_tool_frame").GetTransform()

        def up(x):
          self.robot.SetDOFValues(
            x,
            np.r_[self.robot.GetManipulator("rightarm").GetArmIndices(),
                  self.robot.GetManipulator("leftarm").GetArmIndices()],
            False)

          r_t = self.robot.GetLink("r_gripper_tool_frame").GetTransform()
          l_t = self.robot.GetLink("l_gripper_tool_frame").GetTransform()

          t1 = r_link_t.dot(np.linalg.inv(r_t))
          t2 = l_link_t.dot(np.linalg.inv(l_t))
          r1 = openravepy.quatFromRotationMatrix(t1).tolist()
          r2 = openravepy.quatFromRotationMatrix(t2).tolist()

          return r1[1:3] + r2[1:3]

        for t in xrange(1, n_steps):
          prob.AddConstraint(up, [(t, j) for j in xrange(14)], "EQ", "up%i" % t)

      result = trajoptpy.OptimizeProblem(prob)
      traj = result.GetTraj()
      total_cost = sum(cost[1] for cost in result.GetCosts())
      return traj, total_cost

  def plan_with_pose(self, pose_by_manip,
                     collisionfree=True,
                     init_dofs=None,
                     n_steps=None,
                     init_data=None,
                     maintain_rel=False,
                     maintain_up=False,
                     maintain_linear=False):
    costs = []
    constraints = []
    if 'base' in pose_by_manip.keys():
      # base dofs have different cost from joints
      if init_dofs is None:
        raise Exception("Base motion planning must have init_dofs")

      n_dofs = len(init_dofs)
      cost_coeffs = n_dofs * [5]
      cost_coeffs[-1] = 500
      cost_coeffs[-2] = 500
      cost_coeffs[-3] = 500

      joint_vel_cost = {
        "type": "joint_vel",
        "params": {"coeffs": cost_coeffs}
      }
      costs.append(joint_vel_cost)
    else:
      joint_vel_cost = {
        "type": "joint_vel",  # joint-space velocity cost
        "params": {"coeffs": [5]}  # a list of length one is automatically expanded to a list of length n_dofs
      }
      costs.append(joint_vel_cost)

    for manip, pose in pose_by_manip.items():
      if manip == 'rightarm':
        link = 'r_gripper_tool_frame'
      elif manip == 'leftarm':
        link = 'l_gripper_tool_frame'
      elif manip == 'base':
        link = 'base_footprint'

      if maintain_linear:
        starting_pose = openravepy.poseFromMatrix(self.robot.GetLink(link).GetTransform()).tolist()
        for t, p in enumerate(mu.linspace2d(starting_pose, pose, n_steps)):
          p = p.tolist()
          constraints.append({
            "type": "pose",
            "params": {"xyz": p[4:],
                       "wxyz": pose[:4],  # don't interpolate rotation
                       "link": link,
                       "pos_coeffs": [20, 20, 20],
                       "rot_coeffs": [20, 20, 20],
                       "timestep": t}
          })

      constraints.append({
        "type": "pose",
        "params": {"xyz": pose[4:],
                   "wxyz": pose[:4],
                   "link": link,
                   "pos_coeffs": [20, 20, 20],
                   "rot_coeffs": [20, 20, 20]}
      })

    return self._generate_traj(costs, constraints, n_steps, collisionfree,
                               init_dofs, init_data, maintain_rel, maintain_up)

  def base_plan_multi_init(self, base_pose, init_dofs,
                           collisionfree=True,
                           n_steps=None,
                           init_data=None,
                           maintain_rel=False,
                           maintain_up=False,
                           maintain_linear=False):
    traj_and_cost1 = self.plan_with_pose(
      {'base': base_pose}, collisionfree=collisionfree, init_dofs=init_dofs,
      n_steps=n_steps, init_data=init_data, maintain_rel=maintain_rel,
      maintain_up=maintain_up, maintain_linear=maintain_linear)
    collisions = self.collision_checker.get_traj_collisions(traj_and_cost1[0])

    if collisions == set():
      return traj_and_cost1
    else:  # do trajopt multi-init
      waypoint_step = (n_steps - 1) // 2
      env_min, env_max = utils.get_environment_limits(self.env, robot=self.robot)
      current_dofs = self.robot.GetActiveDOFValues().tolist()
      waypoint_dofs = list(current_dofs)

      MAX_INITS = 10
      n = 0
      while True:
        n += 1
        if n > MAX_INITS:
          print "Max base multi init limit hit!"
          return traj_and_cost1  # return arbitrary traj with collisions

        print "Base planning failed! Trying random init. Iteration {} of {}".format(n, MAX_INITS)

        # randomly sample x, y within env limits + some padding to accomodate robot
        padding = 5.0
        env_min_x = env_min[0] - padding
        env_min_y = env_min[1] - padding
        env_max_x = env_max[0] + padding
        env_max_y = env_max[1] + padding

        waypoint_x = MotionPlanner.state.uniform(env_min_x, env_max_x)
        waypoint_y = MotionPlanner.state.uniform(env_min_y, env_max_y)

        waypoint_dofs[-3] = waypoint_x
        waypoint_dofs[-2] = waypoint_y

        init_data = np.empty((n_steps, self.robot.GetActiveDOF()))
        init_data[:waypoint_step+1] = mu.linspace2d(current_dofs, waypoint_dofs, waypoint_step+1)
        init_data[waypoint_step:] = mu.linspace2d(waypoint_dofs, init_dofs, n_steps - waypoint_step)

        traj_and_cost = self.plan_with_pose(
          {'base': base_pose}, collisionfree=collisionfree, init_dofs=init_dofs,
          n_steps=n_steps, init_data=init_data, maintain_rel=maintain_rel,
          maintain_up=maintain_up, maintain_linear=maintain_linear)
        collisions = self.collision_checker.get_traj_collisions(traj_and_cost[0])
        if collisions != set():
          continue

        return traj_and_cost

  def plan_with_dofs(self, dof_targets,
                     collisionfree=True,
                     n_steps=None,
                     init_data=None):
    costs = [{
      "type": "joint_vel",  # joint-space velocity cost
      "params": {"coeffs": [5]}  # a list of length one is automatically expanded to a list of length n_dofs
    }]
    constraints = [{
      "type": "joint",
      "params": {"vals": dof_targets}
    }]
    return self._generate_traj(costs, constraints, n_steps, collisionfree,
                               dof_targets, init_data, False, False)

  def optimize_traj(self, init_traj, manip, collisionfree=True):
    """
    Takes a given trajectory and optimizes it, keeping the end gripper pose
    the same for the chosen manipulator.
    """
    if manip == 'rightarm':
      link = 'r_gripper_tool_frame'
    elif manip == 'leftarm':
      link = 'l_gripper_tool_frame'

    with self.env:
      orig_values = self.robot.GetDOFValues(self.robot.GetActiveDOFIndices())
      self.robot.SetDOFValues(init_traj[-1], self.robot.GetActiveDOFIndices())
      mat = self.robot.GetLink(link).GetTransform()
      pose = openravepy.poseFromMatrix(mat).tolist()
      # reset
      self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())

    pos = pose[4:]
    rot = pose[:4]
    return self.plan_with_pose(
      pos, rot, collisionfree=collisionfree,
      dof_targets=init_traj[-1], init_data=init_traj, manip=manip,
      n_steps=len(init_traj))
