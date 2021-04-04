DOMAIN_PATH = "../domains/"
errFileName = "robotics_autogen_err1.txt"
ENVPATH="../environments/"

FF = "ff"
MP = "mp"
FD = "fd"
FDOPTIMALMODE = True
FFEXEC = "../planners/FF-v2.3/ff"
FDEXEC = "../planners/FD/src/plan-ipc seq-sat-fd-autotune-1 "
FDOPTEXEC = "../planners/FD/src/plan-ipc seq-opt-lmcut "
MPEXEC = "../planners/M/Mp"
PLANNER_TO_USE = FD

pddlDomainFile = DOMAIN_PATH #+ raw_input("Enter PDDL domain file name: ")
if pddlDomainFile == DOMAIN_PATH:
    #pddlDomainFile = DOMAIN_PATH+ "robotics_obstrn_compiled_dom2.pddl"
    #pddlDomainFile = DOMAIN_PATH+ "robotics_obstrn_compiled_dom2_typed_twoarms.pddl"
  
    # pddlDomainFile = DOMAIN_PATH + "robotics_twoarms_objloc.pddl"
    # pddlDomainFile = DOMAIN_PATH + "robotics_twoarms_objloc_surfaceloc.pddl"
    # pddlDomainFile = DOMAIN_PATH + "robotics_twoarms_objloc_putdownloc_lowmem_dom.pddl"
    # pddlDomainFileNoGeomEff = DOMAIN_PATH+ "robotics_twoarms_objloc_ignore_obs.pddl"


     # for ff, dinnertime:
     # pddlDomainFile = DOMAIN_PATH+ "dinnerTime_dom.pddl"   
     #pddlDomainFile = DOMAIN_PATH+ "dinnerTimeNoNegation_dom.pddl"
    #pddlDomainFile = DOMAIN_PATH+ "dinnerTimeNoNegationCosts_dom.pddl"
    #pddlDomainFile = DOMAIN_PATH + "dinnerTime_twoarms_dom_costs.pddl"
    # pddlDomainFile = DOMAIN_PATH + "dinnerTime_onearm_costs_fast_dom.pddl"
    #pddlDomainFile = DOMAIN_PATH + "dinnerTime_twoarms_compiledcosts_fast_dom.pddl"
    # pddlDomainFile = DOMAIN_PATH + "dinnerTime_twoarms_costs_fast_dom.pddl"
    # pddlDomainFile = DOMAIN_PATH+ "dinnerTimeCompiledCosts_dom.pddl"
  
    pddlDomainFile = DOMAIN_PATH + "dinnerTime_handoff_dom.pddl"
    pddlDomainFileNoGeomEff = pddlDomainFile
  
    # pddlDomainFile = DOMAIN_PATH + "dinnerTime_handoff_fixedheight_dom.pddl"
    # pddlDomainFileNoGeomEff = pddlDomainFile
    
    # pddlDomainFile = DOMAIN_PATH + "drawer_dom.pddl"
    # pddlDomainFileNoGeomEff = pddlDomainFile


initialProblemFile = DOMAIN_PATH #+ raw_input("Enter PDDL problem file name: ")
if initialProblemFile == DOMAIN_PATH:
    #initialProblemFile = DOMAIN_PATH + "robotics_autogen_prob_65objs_typed.pddl"
    #initialProblemFile = DOMAIN_PATH + "canworld_gazebo_12cans_twoarms.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_12cans_prob_objloc.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_22cans_prob_objloc.pddl"
    #initialProblemFile = DOMAIN_PATH + "robotics_twoarms_22cans_objloc_fixedbase_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_50cans_prob_objloc.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_50cans_prob_objloc_surfaceloc.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_objloc_putdownloc_lowmem_50cans_prob.pddl"

    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_22cans_objloc_fixedbase_prob.pddl"

    #for ff, dinnertime:
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_prob.pddl"
    #initialProblemFile = DOMAIN_PATH + "dinnerTimeNoNegation_prob.pddl"
    #initialProblemFile = DOMAIN_PATH + "dinnerTimeNoNegationCostsTargets_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_onearm_costs_fast_prob.pddl"
    #initialProblemFile = DOMAIN_PATH + "dinnerTime_twoarms_compiledcosts_fast_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_twoarms_costs_fast_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTimeCompiledCosts_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_prob.pddl"
    initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_prob_2obj.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_fixedheight_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "drawer_prob.pddl"


collision_free_grasping_samples = 1
occluding_objects_grasping_samples = 1

doJointInterpretation = False
use_ros = False
OpenRavePlanning = False

envFile = ENVPATH+"created_info.dae"
# envFile = ENVPATH + "rll_tray_world.dae"
#envFile = ENVPATH + "rll_dinner_demo_handoff.dae"

DISABLE_BASE = False
DO_BACKTRACKING = True
REPLAN = False
USE_SOFT_LIMITS = False
PRINT_GEN_COUNT = False
USE_MAX_ITERATIONS = False
APPLY_DSH_PATCH = False

if 'surfaceloc' in pddlDomainFile or 'putdownloc' in pddlDomainFile:
    REPORT_PUTDOWN_OBSTRUCTIONS = True
else:
    REPORT_PUTDOWN_OBSTRUCTIONS = False

CAN_DOMAIN = 0
DINNER_DOMAIN = 1
DRAWER_DOMAIN = 2
if 'dinner' in pddlDomainFile:
    DOMAIN = DINNER_DOMAIN
elif 'drawer' in pddlDomainFile:
    DOMAIN = DRAWER_DOMAIN
else:
    DOMAIN = CAN_DOMAIN

# runs hybridPlanner automatically, no raw_inputs, use seed that generation script used, and only motion plan, no actual execution
# second arg is seconds before planning times out
run_test_mode = (True, 65000)

import time
seed = int(time.time())
# seed = 1379030539
if run_test_mode[0]:
    try:
        with open("../tests/seed.txt", 'r') as f:
            seed = int(f.read())
    except (IOError, ValueError):
        print("Could not read seed that generation script used!")

print "SEED: {}".format(seed)
