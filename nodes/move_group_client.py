#!/usr/bin/env python
"""A moveit planner client created for the moveit_commander_timing_problem issue"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

import sys

import rospy

from moveit_commander_timing_problem.srv import PlanGripper, SetGripperOpen

if __name__ == "__main__":

    # Init service node
    rospy.init_node("moveit_planner_client")

    # Initialize moveit_planner_server/set_gripper_open service
    rospy.logdebug("Connecting to 'moveit_planner_server/set_gripper_open' service...")
    rospy.wait_for_service("moveit_planner_server/set_gripper_open")
    try:
        set_gripper_open_srv = rospy.ServiceProxy(
            "moveit_planner_server/set_gripper_open", SetGripperOpen
        )
        rospy.logdebug("Connected to 'moveit_planner_server/set_gripper_open' service.")
    except rospy.ServiceException as e:
        rospy.logerr(
            "Panda_autograsp 'set_gripper_open' service initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            set_gripper_open_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    # Initialize moveit_planner_server/plan_gripper service
    rospy.logdebug("Connecting to 'moveit_planner_server/plan_gripper' service...")
    rospy.wait_for_service("moveit_planner_server/plan_gripper")
    try:
        plan_gripper_srv = rospy.ServiceProxy(
            "moveit_planner_server/plan_gripper", PlanGripper
        )
        rospy.logdebug("Connected to 'moveit_planner_server/plan_gripper' service.")
    except rospy.ServiceException as e:
        rospy.logerr(
            "Panda_autograsp 'moveit_planner_server/plan_gripper' service "
            "initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            plan_gripper_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    # Call set_gripper_open service
    set_gripper_open_srv()

    # Call plan_gripper_service
    plan_gripper_srv()

    # Wait till enter
    input("Click enter to exit> ")
