import StringIO
from PDDLPatcher import *
import sys, subprocess, time, os
import planning_primitives
from HLPlan import *
from settings import *
import getopt
import utils
import re
import copy
import PlannerOps
import shutil
import IPython
totalExecTime = 0
#myPatcher = None


class HybridPlanner:
    def __init__(self, pddlDomainFile, pddlDomainFileNoGeomEff, 
                 initialProblemFile, viewer, envFile, planner = FF):
        self.pid = repr(os.getpid())
        self.initialProblemFile = initialProblemFile
        self.editedProblemFile = DOMAIN_PATH+"incremental_temp_" + \
            initialProblemFile.replace(DOMAIN_PATH, "").replace(".pddl",  \
                                                                    "_" + self.pid + ".pddl")
        shutil.copyfile(self.initialProblemFile, self.editedProblemFile)
        tempFileList = [fname for fname in os.listdir(DOMAIN_PATH) if "incremental_temp_" in fname]
        print "Number of temp files: " + repr(len(tempFileList))
        if len(tempFileList) > 25:
            print "\n\n"
            print "Consider cleaning up using: \n rm " + DOMAIN_PATH+"incremental_temp_*"
            print "\n\n"
            time.sleep(2)

        self.pddlDomainFile = pddlDomainFile
        self.pddlDomainFileNoGeomEff = pddlDomainFileNoGeomEff
        self.iteration = 0
        self.ORSetup = planning_primitives.initOpenRave(viewer, envFile)

        # if run_test_mode[0]:
        #     pass
        # else:
        #     if 'robotics' in self.pddlDomainFile:
        #         goalObject = raw_input("Enter object to pick up, or press return for default:")
        #         if len(goalObject) > 0:
        #             self.editedProblemFile = utils.setGoalObject(goalObject, self.editedProblemFile)
                
        self.problemFileList = [self.editedProblemFile]
        self.patcher = PDDLPatcher(self.editedProblemFile)
        self.plannerName = planner
        self.planList = []
        self.currentPlanIndex = -1
        self.cacheClearCount = 0
    
    
    def updatePlan(self, generatedPlan):
        self.planList.append(generatedPlan)
        self.currentPlanIndex += 1
        

    def getCurrentPlan(self):
        return self.planList[self.currentPlanIndex]

        
    def iterativePlan(self):
        iteration = 0
        startTime = time.time()
        prevPDDLFile = None
        saved_info = None
        pddlProblemFile = self.editedProblemFile
        oldPlanPrefix = ""
        oldPlan = None
        origState = None
        failureStep = None
        replanMode = False
        resumeFrom = 0
        while True:
            reinterpreted = False
            self.iteration += 1
            plannerOutFname = pddlProblemFile + ".out"
            if self.iteration > 1:
                replanMode = True
            strPlanFileH, plannerOutStr, planCount = self.runPlanner(pddlProblemFile, 
                                                            plannerOutFname, replanMode)
            if strPlanFileH ==-1:
                reinterpreted = terminateOrReinterpret()
                generatedPlan = self.getCurrentPlan()
                print "Retrying Plan (from {0}):".format(resumeFrom)
                generatedPlan.printPlan()
            else:
                saved_info = (copy.copy(strPlanFileH), plannerOutStr, planCount,
                              self.iteration, plannerOutFname)
                stateList = []
                if self.plannerName == MP or self.plannerName == FD:
                    stateList = self.planner.getStateList()

                generatedPlan = HLPlan(self.pddlDomainFile, self.pddlDomainFileNoGeomEff,
                                       pddlProblemFile, strPlanFileH, plannerOutStr,
                                       self.plannerName, planCount, stateList, 
                                       origState)
                print "Generated plan: "
                generatedPlan.printPlan()
                print "Trimmed plan:"
                generatedPlan.trimMoves()
                generatedPlan.printPlan()

                if oldPlan != None:
                    resumeFrom = generatedPlan.incorporate(oldPlan, failureStep)
                    print "Need to resume refinement from step " + repr(resumeFrom) + " in:" 
                    generatedPlan.printPlan()
                self.updatePlan(generatedPlan)

            strPlanFileH = generatedPlan.getStrIOPlan()

            print "\nWill try to pick objects in order: "+ \
                repr(getObjSeqInPlan(strPlanFileH, 'object'))+"\n"

            success = False
            usefulErrors = False
            while not usefulErrors:       
                try:
                    self.tryMotionPlanning(strPlanFileH, reinterpreted, resumeFrom, startTime)
                    sys.exit(0)
                except planning_primitives.ActionError, e:
                    errorStr = e.pddl_error_info
                    failureStep = self.getFailedActionNumberAndProps(errorStr)[0]
                    strPlanFileH.seek(0)
                    usefulErrors = True
                    self.iteration += 1
                except planning_primitives.InstantiationExhaustedException as e:
                    if not APPLY_DSH_PATCH:
                        raise e
                    if "instantiations exhausted" in e.args[0]:
                        print "\n\nResetting states\n\n"
                        self.iteration += 1
                        ## forget all learned facts, try again:
                        newPlan = copy.deepcopy(generatedPlan)
                        newPlan.resetStates()
                        self.updatePlan(newPlan)
                        planning_primitives.clearGPCache(self.ORSetup)
                        prevPDDLFile = pddlProblemFile
                        pddlProblemFile = self.editedProblemFile.replace(".pddl",
                                                                         "_" + 
                                                                         repr(self.iteration) + 
                                                                         ".pddl")
                        newPlan.writeInitFile(0, prevPDDLFile, pddlProblemFile)
                        generatedPlan = newPlan
                        resumeFrom = 0
                        reinterpreted = False
                        strPlanFileH = generatedPlan.getStrIOPlan()

            if errorStr == "":
                print "Lower level failed without error message. Possibly due to sampling limit."
                sys.exit()

            print "Got facts: \n"+errorStr+"\n"

            prevPDDLFile = pddlProblemFile
            pddlProblemFile = self.editedProblemFile.replace(".pddl", "_" + repr(self.iteration) + ".pddl")
            oldPlan = copy.deepcopy(generatedPlan)
            failureStep, propList = self.getFailedActionNumberAndProps(errorStr)
            origState = copy.deepcopy(oldPlan.origStateList[failureStep])
            oldPlan.patchState(failureStep, propList)
            oldPlan.writeInitFile(failureStep, prevPDDLFile, pddlProblemFile)
            oldPlan.truncateFrom(failureStep)

  
    def tryMotionPlanning(self, strPlanFileH, reinterpreted, resumeFrom, startTime):
        planning_primitives.test(strPlanFileH, self.ORSetup, objList=[],
                                 reinterpreted=reinterpreted, 
                                 resumeFrom=resumeFrom, startTime=startTime)
        print "Success. Quitting."
        print "Replan count: "+ repr(self.iteration)
        print "Cache clearing count: "+ repr(self.cacheClearCount)
        endTime = time.time()
        print "Execution took " + repr(endTime-startTime) + " seconds"
        if run_test_mode[0]:
            sys.exit(0)
        else:
            raw_input("Hit 'Enter' to close.")
            sys.exit(0)


    def getFailedActionNumberAndProps(self, errorStr):
        errorLines = errorStr.lower().split("\n")
        failedActionNumber = int(errorLines[0].strip("linenumber: "))
        propList = filter(lambda x:len(x) > 0, errorLines[1:])
        return failedActionNumber, propList


    def runPlanner(self, pFile, oFile, replanMode):
        if self.plannerName == FF:
            self.planner = PlannerOps.FF(pddlDomainFile = self.pddlDomainFile,
                                    pddlProblemFile = pFile,  
                                    outputFile = oFile)    
        if self.plannerName == FD:
            self.planner = PlannerOps.FD(replanMode = replanMode, pddlDomainFile = self.pddlDomainFile,
                                    pddlProblemFile = pFile,   
                                    outputFile = oFile)
        if self.plannerName == MP:
            self.planner = PlannerOps.MP(pddlDomainFile = self.pddlDomainFile,
                                    pddlProblemFile = pFile,   
                                    outputFile = oFile)
        return self.planner.getResult()


# def forgetAndRestart(myPatcher, learnedPredicate, problemFile, executor, cacheClearCount):
#     cacheClearCount += 1
#     myPatcher.forgetLearnedFactsAbout("obstructs")
#     myPatcher.writeCurrentInitState(problemFile)
#     planning_primitives.clearGPCache(executor)
#     return cacheClearCount


def terminateOrReinterpret():
    return True
    if run_test_mode[0]:
        reinterpret = 'y'
    else:
        reinterpret = raw_input("Planner failed. Reinterpret (y/n)?")
    if reinterpret == 'n':
        print "Quitting"
        sys.exit(-1)
    return True
      
            
def getObjSeqInPlan(file_object_or_name, objectNamePrefix = 'object'):
        objSeq = []
        if type(file_object_or_name) is str:
            file_obj = open(file_object_or_name)
        else:
            file_obj = file_object_or_name
            
        planStr = file_obj.read().lower()
        planLines = planStr.strip().split("\n")
        for line in planLines:
            if 'grasp' in line:
                fixedWLine = re.sub(r' +', ' ', line)
                objMatch = re.search('grasp\w* \w+', fixedWLine)
                obj = objMatch.group().split()[1]
                objSeq.append(obj)
                
        file_obj.seek(0)
    
        return objSeq

def run_with_ros(envFile, viewer=True):
    planning_primitives.use_ros = True
    hp = HybridPlanner(pddlDomainFile, pddlDomainFileNoGeomEff, 
                       initialProblemFile, viewer, envFile, planner = FD)
    hp.iterativePlan()


if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:],"vme:")
    except getopt.GetoptError:
        print("Use -v for viewer, -m to use MP (FF default), -e for env file name")
        sys.exit(2)

    viewer = False
    planner = PLANNER_TO_USE # in settings
    for opt,arg in opts:
        if opt == "-v":
            print "Setting viewer mode to True"
            viewer = True
        elif opt == "-m":
            print("Using MP instead of FF")
            planner = MP
        elif opt == "-e":
            print("Using env file: %s" %arg)
            envFile = arg
             
    if run_test_mode[0]:
        hp = HybridPlanner(pddlDomainFile, pddlDomainFileNoGeomEff,
                           initialProblemFile, viewer, envFile, planner=planner)
        hp.iterativePlan = utils.timeout(seconds=run_test_mode[1])(hp.iterativePlan)
        try:
            hp.iterativePlan()
        except utils.TimeoutError as e:
            print("Timed out: %s" %e)
            sys.exit(1)
    else:
        hp = HybridPlanner(pddlDomainFile, pddlDomainFileNoGeomEff, 
                           initialProblemFile, viewer, envFile, planner=planner)
        hp.iterativePlan()



    #pddlDomainFile = '../domains/dinnerTimeNoNegationCosts_dom.pddl'
    #pddlProblemFile = '../domains/dinnerTimeNoNegationCosts_prob.pddl'
    #plan, fdOutStr = runPlannerFD(pddlDomainFile, pddlProblemFile)
    #op = OutputParser("")
    #myPatcher.patchWithFDOutput(fdOutStr, 10)
    #propList = ["(not (smaller random_object11 random_object12))"]
    #myPatcher.patchWithProps(propList)
