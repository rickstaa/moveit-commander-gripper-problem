#!/usr/bin/env python
"""A moveit planner server created for the moveit_commander_gripper_problem issue"""

import rospy
import sys

from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import ApplyPlanningScene

import moveit_commander
from moveit_commander import MoveItCommanderException
from moveit_commander_gripper_problem.srv import PlanGripper, SetGripperOpen


class MoveGroupTest:
    """Move it planner server class"""

    def __init__(self):

        # Connect to moveit services
        rospy.loginfo(
            "Connecting moveit default moveit 'apply_planning_scene' " "service."
        )
        rospy.wait_for_service("apply_planning_scene")
        try:
            self._moveit_apply_planning_srv = rospy.ServiceProxy(
                "apply_planning_scene", ApplyPlanningScene
            )
            rospy.loginfo("Moveit 'apply_planning_scene' service found!")
        except rospy.ServiceException as e:
            rospy.logerr(
                "Moveit 'apply_planning_scene' service initialization failed: %s" % e
            )
            shutdown_msg = (
                "Shutting down %s node because %s service connection failed."
                % (rospy.get_name(), self._moveit_apply_planning_srv.resolved_name)
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        # Get robot commander
        self.robot = moveit_commander.RobotCommander(
            robot_description="robot_description", ns="/"
        )

        # Get move_group
        self.move_group_gripper = self.robot.get_group("hand")
        self.move_group_gripper.set_planner_id("TRRTkConfigDefault")

        # Created services
        rospy.Service(
            "%s/set_gripper_open" % rospy.get_name()[1:],
            SetGripperOpen,
            self.set_gripper_open_service,
        )
        rospy.Service(
            "%s/plan_gripper" % rospy.get_name()[1:],
            PlanGripper,
            self.plan_gripper_service,
        )

        # Initialize member variables
        self.desired_gripper_joint_values = {}

    def set_gripper_open_service(self, req):
        """Set gripper joint targets values to open.

        ----------
        Parameters
        req :  :py:obj:`panda_autograsp.msg.SetGripperOpen.`
            Empty service request.

        Returns
        -------
        bool
                Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper "
                "can not be controlled."
            )
            return False

        # Get gripper target value to open
        try:
            # Get desired joint targets
            desired_joint_values = self.move_group_gripper.get_named_target_values(
                "open"
            )

            # Save desired joint targets
            self.desired_gripper_joint_values = desired_joint_values
            return True
        except MoveItCommanderException as e:
            rospy.logwarn(e)
            return False

    def plan_gripper_service(self, req):
        """Compute plan for the currently set gripper target joint values.

        Parameters
        ----------
        req : :py:obj:`panda_autograsp.msg.PlanGripper.`
            Empty service request.

        Returns
        -------
        bool
            Returns a bool to specify whether the plan was executed successfully.
        """

        # Check if gripper controller exists
        if self.move_group_gripper is None:
            rospy.logwarn(
                "Gripper move group appears to be missing. As a result the gripper can "
                "not be controlled."
            )
            return False

        # Plan for gripper command
        desired_values = self.desired_gripper_joint_values

        # DEBUG: Uncomment and comment this line
        # rospy.sleep(0.1)
        # DEBUG: Uncomment and comment this line

        plan = self.move_group_gripper.plan(desired_values.values())
        print("Plan length: %i" % len(plan.joint_trajectory.points))

        # Validate whether planning was successful
        if plan_exists(plan):
            rospy.loginfo("Gripper plan found.")
            return True  # Return success bool
        elif at_joint_target(
            self.move_group_gripper.get_current_joint_values(),
            self.desired_gripper_joint_values.values(),
            self.move_group_gripper.get_goal_joint_tolerance(),
        ):  # Plan empty because already at goal
            rospy.loginfo("Gripper already at desired location.")
            return True
        else:  # Plan empty
            rospy.logwarn("No gripper plan found.")
            return False


def at_joint_target(current, desired, goal_tolerance):
    """This function can be used to check if the move_group is already at the desired
    joint target.

    Parameters
    ----------
    current : :py:obj:`list`
        The current joint configuration.
    desired : :py:obj:`list`
        The desired joint target.
    goal_tolerance : :py:obj:`float`
        The planner target goal tolerance.

    Returns
    -------
    :py:obj:`bool`
        Bool specifying if move_group is already at the goal target.
    """

    # Round to the goal tolerance
    accuracy = str(goal_tolerance)[::-1].find(".")
    current = [round(item, accuracy) for item in current]
    desired = [round(item, accuracy) for item in desired]

    # Check if move_group is at joint target
    if current == desired:
        return True  # Already at goal
    else:
        return False  # Not already at goal


def get_trajectory_duration(trajectory):
    """This function returns the duration of a given moveit trajectory.

    Parameters
    ----------
    trajectory : :py:obj:`!moveit_msgs.msg.DisplayTrajectory`
        The computed display trajectory.

    Returns
    -------
    :py:obj:`float`
        Trajectory duration in seconds.
    """

    # Test if trajectory is given as input
    if not isinstance(trajectory, DisplayTrajectory):
        raise TypeError(
            "Argument save must be of type bool, not {type}".format(
                type=type(trajectory)
            )
        )

    # initiate duration parameters
    duration = 0

    # Loop through trajectory segments
    for test in trajectory.trajectory:

        # Retrieve the duration of each trajectory segment
        if len(test.joint_trajectory.points) >= 1:
            duration += test.joint_trajectory.points[-1].time_from_start.to_sec()
        if len(test.multi_dof_joint_trajectory.points) >= 1:
            duration += test.multi_dof_joint_trajectory.points[
                -1
            ].time_from_start.to_sec()

    # Return duration in seconds
    return duration


def plan_exists(plan):
    """This function can be used to check if a plan trajectory was computed.

    Parameters
    ----------
    plan : :py:obj:`!moveit_msgs.msg.RobotTrajectory`
        The computed robot trajectory.

    Returns
    -------
    :py:obj:`bool`
        Bool specifying if a trajectory is present
    """

    # Check if a trajectory is present on the plan object
    if not all(
        [
            not (
                len(plan.joint_trajectory.points) >= 1
            ),  # True when no trajectory was found
            not (
                len(plan.multi_dof_joint_trajectory.points) >= 1
            ),  # True when no trajectory was found
        ]
    ):
        # A trajectory was found
        return True
    else:

        # No trajectory was found
        return False


if __name__ == "__main__":

    # Init service node
    rospy.init_node("moveit_planner_server")

    # initialize moveit_commander and robot commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create move_group_test object
    move_group_test = MoveGroupTest()

    # Spin forever
    rospy.spin()
