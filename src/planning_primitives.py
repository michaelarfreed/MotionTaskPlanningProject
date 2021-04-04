import openravepy
import utils
import inspect
import generate_reaching_poses
import reachability
import time
import tray_world
import numpy as np
from settings import *
import sys
import pdb
import numpy
from pygraph.classes.digraph import digraph
from pygraph.algorithms.accessibility import accessibility

from action_executor import ActionExecutor, ActionError, InstantiationExhaustedException
from env_manager import EnvManager

try:
    from tf import transformations
    from geometry_msgs.msg import PoseStamped

except:
    print "Warning: ROS imports failed. Okay if not using ROS."


class Executor(object):
    def __init__(self, robot, viewer = False):
        self.robot = robot

        v = self.robot.GetActiveDOFValues()
        v[self.robot.GetJoint('torso_lift_joint').GetDOFIndex()] = 0.3
        self.robot.SetDOFValues(v)

        self.env = robot.GetEnv ()
        self.grasping_locations_cache = {}
        self.viewMode = viewer
        self.tray_stack = []
        self.unMovableObjects = {self.env.GetKinBody('table'),
                                 self.env.GetKinBody('table6'),
                                 self.env.GetKinBody('walls'),
                                 self.env.GetKinBody('drawer_outer'),
                                 self.env.GetKinBody('drawer')}
        self.objSequenceInPlan = []
        self.handled_objs = set()

        self.action_executor = ActionExecutor(self.env, use_ros, self.unMovableObjects)

        self.obstruction_digraph = digraph()
        
        #loading the IK models
        # utils.pr2_tuck_arm(robot)
        robot.SetActiveManipulator('leftarm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
                        robot,iktype=openravepy.IkParameterization.Type.Transform6D)    
        if not ikmodel.load():
            ikmodel.autogenerate()            
        robot.SetActiveManipulator('rightarm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot,iktype=openravepy.IkParameterization.Type.Transform6D)    
        if not ikmodel.load():
            ikmodel.autogenerate()

    def getGoodBodies(self):
        if not doJointInterpretation:
            body_filter = lambda b: b.GetName().startswith("random") or\
                                    b.GetName().startswith('object')
        else:
            futureObjects = set(self.objSequenceInPlan) - self.handled_objs
            body_filter = lambda b: (b.GetName() not in futureObjects) \
                                   and (b.GetName() not in self.unMovableObjects)
        return filter(body_filter, self.env.GetBodies())

    def get_bad_bodies(self, obj_to_grasp):
        obj_name = obj_to_grasp.GetName()

        if doJointInterpretation:
            obstructions = accessibility(self.obstruction_digraph).get(obj_name, None)

        future_objects = []
        if doJointInterpretation and obstructions is not None:
            future_objects = obstructions
            future_objects.remove(obj_name)

        # print self.obstruction_digraph
        # print obj_name, future_objects
        # raw_input("!!")
        bad_body_filter = lambda b: (b.GetName() in future_objects) \
                                 or (b.GetName() in self.unMovableObjects)
        return set(filter(bad_body_filter, self.env.GetBodies()))

    def setObjSequenceInPlan(self, objList):
        self.objSequenceInPlan = objList
        print
        print "Will try to pick objs in the order " + repr(objList)

    def clear_gp_cache(self):
        self.grasping_locations_cache = {}
        self.objSequenceInPlan = []
        if not DO_BACKTRACKING:
            self.action_executor.reset_all_actions()

    def pause(self, msg=None):
        if self.viewMode:
            if msg is None:
                raw_input("Press return to continue")
            else:
                print msg
                #time.sleep(0.5)
                raw_input(msg + "... [press return]")

    def moveto(self, lineno, _unused1, pose, req_side=None):
        if type(pose) is str:
            args = [arg.lower() for arg in pose.split('_')]

            if 'arm' in args[0]:
                moveto_type = args[1]
            else:
                arm = 'rarm'
                moveto_type = args[0]

            if moveto_type == 'gp' or moveto_type == 'blf':
                if 'arm' in args[0]:
                    # Ex: RARM_GP_OBJECT7
                    arm = args[0]
                    obj_name = args[2]
                else:
                    obj_name = args[1]

                ###TODO: gp and blf are not the same!!
                obj = self.env.GetKinBody(obj_name)
                self.action_executor.add_moveto_gp(
                    lineno, obj, manip=utils.to_manip(arm),
                    req_side=req_side, pose_name=pose)
            elif moveto_type == 'pdp':
                if 'arm' in args[0]:
                    if REPORT_PUTDOWN_OBSTRUCTIONS:
                        # Ex: LARM_PDP_TABLE6_LOC1
                        arm = args[0]
                        table_name = args[2]
                        loc_name = args[3]

                        obj = None
                        table = self.env.GetKinBody(table_name)
                    else:
                        # Ex: LARM_PDP_OBJECT7_TABLE6
                        arm = args[0]
                        obj_name = args[2]
                        table_name = args[3]

                        obj = self.env.GetKinBody(obj_name)
                        table = self.env.GetKinBody(table_name)
                        loc_name = None
                else:
                    obj_name = args[1]
                    table_name = args[2]

                    obj = self.env.GetKinBody(obj_name)
                    table = self.env.GetKinBody(table_name)
                    loc_name = None

                ###BUG here: table_name may not be known
                self.action_executor.add_moveto_pdp(
                    lineno, obj, table, loc_name,
                    req_side=req_side, pose_name=pose)
            else:
                raise Exception("Bad moveto action: {}".format(pose))

    def moveto_left(self, lineno, _unused1, pose_name, _unused2):
        # Move, when the object must be on the left.
        self.moveto(lineno, _unused1, pose_name, req_side="left")

    def moveto_right(self, lineno, _unused1, pose_name, _unused2):
        # Move, when the object must be on the right.
        self.moveto(lineno, _unused1, pose_name, req_side="right")

    def grasp(self, lineno, obj_name, _unused1, pose_name, gripper='rgripper', _unused2=None):
        obj = self.env.GetKinBody(obj_name)
        self.action_executor.add_pickup(lineno, obj, manip=utils.to_manip(gripper), pose_name=pose_name)

    def putdown(self, lineno, obj_name, location_str, pose_name, gripper='rgripper'):
        obj = self.env.GetKinBody(obj_name)
        self.action_executor.add_putdown(lineno, obj, location_str, manip=utils.to_manip(gripper), pose_name=pose_name)

    def find_gp(self, object_to_grasp):
        """"Find a grasping pose
        """""

        print "Getting a collision free grasping pose"
        robot_pose, _, _= generate_reaching_poses.get_collision_free_grasping_pose(
                                                                                               self.robot, 
                                                                                               object_to_grasp, 
                                                                                               )
        return robot_pose

    def find_bp(self, surface):
        """""Find a base pose for putting down
        """""

        print "Getting base pose for putting down"
        robot_pose, _, _= generate_reaching_poses.get_collision_free_surface_pose(self.getGoodBodies(),
                                                                                                       self.robot, 
                                                                                                       surface, 
                                                                                                       )
        return robot_pose

    def placeontray(self, lineno, pose_name, obj_name, unused2, tray_name, unused4, gripper='rgripper', unused5=None, unused6=None, unused7=None):
        """Put an item on the tray.
        """
        self.putdown(lineno, obj_name, tray_name, gripper=gripper, pose_name = pose_name)

    def picktray(self, lineno, pose_name, tray_name, unused2):
        """Grabs the tray and all the items on it
        """
        tray = self.env.GetKinBody(tray_name)
        self.action_executor.add_pickup_tray(lineno, tray, pose_name)

    def putdowntray(self, lineno, pose_name, tray_name, tray_loc):
        """Move the robot to the tray goal location and releases the tray
        """
        tray = self.env.GetKinBody(tray_name)
        tray_loc = self.env.GetKinBody(tray_loc)
        self.action_executor.add_putdown_tray(lineno, tray, tray_loc, pose_name)

    def pickfromtray(self, lineno, pose_name, tray_name, obj_name, unused2, unused3, gripper='rgripper', unused4=None, unused5=None):
        self.grasp(lineno, obj_name, None, gripper=gripper, pose_name = pose_name)

    def handoff_l_r(self, lineno, obj_name):
        self.action_executor.add_handoff(lineno, "leftarm", obj_name)

    def handoff_r_l(self, lineno, obj_name):
        self.action_executor.add_handoff(lineno, "rightarm", obj_name)

    def opendrawer(self, lineno, drawer_name, pose_name, gripper='rgripper'):
        drawer = self.env.GetKinBody(drawer_name)
        self.action_executor.add_open_drawer(lineno, drawer, manip=utils.to_manip_better(gripper), pose_name=pose_name, open=True)

    def closedrawer(self, lineno, drawer_name, pose_name, gripper='rgripper'):
        drawer = self.env.GetKinBody(drawer_name)
        self.action_executor.add_open_drawer(lineno, drawer, manip=utils.to_manip_better(gripper), pose_name=pose_name, open=False)

class PlanParser(object):
    def __init__(self, file_object_or_name, executor):
        if type(file_object_or_name) is str:
            file_obj = open(file_object_or_name)
        else:
            file_obj = file_object_or_name
        
        isinstance(executor, Executor)
        self.executor = executor   
        
        self.grasping_locations = set()
        self.bindings = {}
        self.base_locations = set()
        self.need_binding = set()
        self.parsing_result = None
        
        self.parse(file_obj)
        
        self.handled_objs = set()

        self.action_executor = self.executor.action_executor
    
    def updateHandledObjs(self, args):
        self.executor.handled_objs = self.handled_objs
        return
    
    def parse(self, file_obj): 
        functions = []
        for l in file_obj:            
            function = {}
            stripped_line =  l.strip(" \t\n")
            
            if stripped_line.startswith('#'):
                continue
            if len(stripped_line) < 2:
                continue
            
            tokens_list = stripped_line.split(" ")[1:] #skip instruction number
            function_name = tokens_list[0].lower()
            if 'move' in function_name:
                splitted = function_name.split("_")
                if len(splitted) == 4:
                  function_name = "_".join([splitted[0], splitted[3]])
                else:
                  function_name = "moveto"
            
            method = getattr(self.executor, function_name, None)
            if method is None:
                raise TypeError("Object %s does not have a method called %s" %(
                    self.executor.__class__.__name__,
                    function_name))
            
            function['name'] = function_name
            function['method'] = method
            function['args'] = []
            num_args = len(inspect.getargspec(method).args[1:])  # remove self
            default_args = inspect.getargspec(method).defaults
            if default_args is not None:
                num_args -= len(default_args)  # remove default args
            num_variables = len(tokens_list[1:])  # remove function name
            num_variables += 1  # also passing in lineno
            if num_args > num_variables:
                raise TypeError("Wrong arguments for %s, expected %d, got %d" %(
                    function_name, num_args, num_variables))
            
            args = (t.strip("\n")
                    for t in tokens_list[1:] if len(t) > 0)
            for arg in args:
                #grasping location ignored!
                if arg.startswith("gp_"):
                    pass
                    #self.grasping_locations.add(arg)
                    #self.need_binding.add(arg)
                #base location
                elif arg.startswith("blf_"):
                    self.base_locations.add(arg)
                    self.need_binding.add(arg)
                
                function['args'].append(arg.lower())
            
            functions.append(function)
        self.parsing_result = functions

    def bind_grasping_location(self, var):
        obj_label = "_".join(var.split('_')[1:])
        obj = self.executor.env.GetKinBody(obj_label)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_label)
        
        pose =  self.executor.find_gp(obj)
        self.bindings[var] = pose
        return pose

    def bind_base_location(self, var):
        table_label = "_".join(var.split('_')[1:])
        table = self.executor.env.GetKinBody(table_label)
        if table is None:
            raise ValueError("Object %s does not exist" % table_label)
        
        pose = self.executor.find_bp(table)
        self.bindings[var] = pose
        return pose
        
    def bind_variable(self, var):
        if var not in self.need_binding:
            return var
        
        elif var in self.bindings:
            return self.bindings[var]

        elif var in self.base_locations:
            return self.bind_base_location(var)

        elif var in self.grasping_locations:
            return self.bind_grasping_location(var)

    def execute(self, timestep=0.5, reinterpreted=False, resumeFrom=0, startTime=None):
        print "Starting..."
        if reinterpreted:
            print "Doing reinterpretation..."
        else:
            print "New partial plan..."
            self.action_executor.prep_partial(resume_from=resumeFrom)

            for lineno, instruction in enumerate(self.parsing_result):
                action_name = instruction["name"] + "(" + ", ".join( instruction['args']) + ")"
                print "Adding: {}".format(action_name)

                if action_name == 'grasp':
                    self.updateHandledObjs(args)

                method = instruction['method']
                args = [self.bind_variable(a) for a in instruction['args']]
                method(*(lineno,) + tuple(args))

        try:
            self.action_executor.get_next_instantiation()
        except ActionError, e:
            self.handle_error(e)
            raise e

        print("Total planning time: " + repr(time.time() - startTime) + " seconds")
        if not run_test_mode[0]:
            self.action_executor.execute_all()
        print "Done"

    def get_error_manip(self, error):
        manip = error.manip
        if manip == 'rightarm':
            arm = 'rarm'
            gripper = 'rgripper'
        elif manip == 'leftarm':
            arm = 'larm'
            gripper = 'lgripper'
        return arm, gripper    


    def handle_error(self, error):
        assert isinstance(error, ActionError)
        robot = error.robot
        obj = error.object_to_grasp
        
        print "Handling an error"
        if "collision" in error.problem:
            print "Got a collision error, finding occlusions"
            pose = None
            torso_angle = None
            collision_list = error.collisions

            object_to_grasp_name = error.object_to_grasp.GetName()
            arm, gripper = self.get_error_manip(error)
            obst_list = "\n".join("(obstructs {} {} {})".format(
                error.pose_name, obstr, object_to_grasp_name) for obstr in collision_list)

            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number, obst_list)
            return

        elif "putdown" in error.problem:
            print "Got a collision during putdown!"
            collision_list = error.collisions
            arm = self.get_error_manip(error)
            msg = "\n".join("(putdownobstructs {} {} {})".format(
                error.pose_name, obj, error.location) for obj in collision_list)
            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number, msg)
            # pdb.set_trace()
            return
            
        elif "unreachable" in error.problem:
            print "Got an unreachable error!"
            arm,gripper = self.get_error_manip(error)
            msg = "(not (IsGP {0} {1} {2}))".format(error.pose_name,
                obj.GetName(), gripper)
            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number, msg)
            return
        elif "heavy" in error.problem:
            print "They tray is too heavy!"
            msg = "(Heavy %s)" % error.object_to_grasp.GetName()
            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number,
                                                          msg )
            return
        elif "Incompatible" in error.problem:
            print "Objects are incompatible!"
            #msg = "(Bigger %s %s)" % (error.object_to_grasp.GetName(),
             #                         error.stacktop.GetName())
            msg = "(not (smaller %s %s))" % (error.object_to_grasp.GetName(),
                                             error.stacktop.GetName())

            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number,
                                                          msg )
        elif "onleft" in error.problem or "onright" in error.problem:
          error.pddl_error_info = "Linenumber: %d\n%s" %(error.line_number, error.problem)
        elif 'ROS pickup' in error.problem:
            print 'ROS pickup failed!'
        elif 'ROS putdown' in error.problem:
            print 'ROS putdown failed!'
        else:
            print "Don't know how to handle this problem!"

            
def initOpenRave(viewer = False, envFile = None):
    env = openravepy.Environment()
    env.StopSimulation()
    if viewer: 
        env.SetViewer('qtcoin')

    env.Load(envFile);
    robot = env.GetRobots()[0];
    manip = robot.SetActiveManipulator('rightarm')
    EnvManager.init_openrave_state(env)
    if DOMAIN == DRAWER_DOMAIN:
      EnvManager.setup_drawer_object_groups(env)
      EnvManager.drawer_init_x = env.GetKinBody('drawer').GetTransform()[0, 3]
    ex = Executor(robot, viewer)
    return ex
        

def test(planFName, ex, objList, reinterpreted=False, resumeFrom=0, startTime=None):
    parser = PlanParser(planFName, ex);
    #ex.setObjSequenceInPlan(objList)
    
    try:
        parser.execute(timestep=0, reinterpreted=reinterpreted, resumeFrom=resumeFrom, startTime=startTime)
        print "All ok"
    except ActionError, e:
        raise 

def clearGPCache(ex):
    print "clearing gp cache"
    ex.clear_gp_cache()
    
    
if __name__ == "__main__":
    global envFile
    initOpenRave(envFile)

    try:
        test("wrongplan.txt")
    except ActionError, e:
        print "Got an execution problem: ", e
        print "PDDL message is: ", e.pddl_error_info
        
    raw_input("Press a button to continue")
