#! /usr/bin/python

import roslib
roslib.load_manifest('perception_manipulation')

import rospy
import openravepy

import hybridPlanner
import openrave_input

from settings import * 
ENV_FILE_NAME = envFile


if __name__=="__main__":
    rospy.init_node('planner_wrapper', anonymous=False)

    # # load new environment
    # env = openravepy.Environment()
    # detector_and_cluster_map = openrave_input.create_openrave_bodies(env, False)

    # # add custom table
    # openrave_input.add_block(env, 'table6', 0, -0.8, 0.5, 0.5, 0.25, 0.005)
    # env.Save(ENV_FILE_NAME)

    env = openrave_input.add_openrave_bodies_and_ar_markers(
        viewer=False, pddl_domain_file=pddlDomainFile)
    env.Save(ENV_FILE_NAME)

    # run planner
    hybridPlanner.run_with_ros(ENV_FILE_NAME, True)
