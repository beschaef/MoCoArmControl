#! /usr/bin/env python

from __future__ import print_function
import rospy, sys

# Brings in the SimpleActionClient
import actionlib

from nxt_action_msgs.msg import ArmPositionAction, ArmPositionGoal

def arm_position_client():
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('ArmPosition', ArmPositionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    #goal = ArmPositionGoal(joint_names_goal = ['motor_1', 'motor_2', 'motor_3'], joint_position_angles_goal=[5.0, 3.0, 4.0])
    goal = ArmPositionGoal(joint_names_goal=['motor_2'], joint_position_angles_goal=[2.57])

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('arm_position_client')
        result = arm_position_client()
        print("Result:", ', '.join([str(n) for n in result.joint_position_angles_result]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

    exit()
