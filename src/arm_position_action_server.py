#!/usr/bin/env python

import roslib;
import rospy
from actionlib import *
from actionlib_msgs.msg import *
from nxt_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from nxt_action_msgs.msg import ArmPositionFeedback, ArmPositionResult, ArmPositionAction

import time

MOTOR_A = "motor_1"
MOTOR_B = "motor_2"
GRIPPER = "motor_3"

class ArmPositionServer:
    _feedback = ArmPositionFeedback()
    _result = ArmPositionResult()

    def __init__(self, name):
        self._sas = SimpleActionServer(name, ArmPositionAction, execute_cb=self.execute_cb, auto_start=False)
        self.arm_joint_pub = rospy.Publisher("nxt_1/joint_command", JointCommand, queue_size=10)
        self.arm_state_sub = rospy.Subscriber("nxt_1/joint_state", JointState, self.joint_states_cb)

        self.efforts = {MOTOR_A: 0.7, MOTOR_B: 0.7, GRIPPER: -0.7}
        self.joint_positions = {MOTOR_A: 0.0, MOTOR_B: 0.0, GRIPPER: 0.0}
        
        self.goal_threshold = 0.2

        self.joint_0_angle = 0.0
        self.joint_0_vel = 0.0
        self.joint_0_eff = 0.0

        self.joint_1_angle = 0.0
        self.joint_1_vel = 0.0
        self.joint_1_eff = 0.0

        self.gripper_angle = 0.0
        self.gripper_vel = 0.0
        self.gripper_eff = 0.0

        self._sas.start()

    def execute_cb(self, msg):
        for i in range(0, len(msg.joint_names_goal)):
            cmd = JointCommand()
            cmd.name = msg.joint_names_goal[i]
            cmd.effort = self.efforts[msg.joint_names_goal[i]]
            rospy.logwarn(str(msg.joint_names_goal[i]))
            rospy.logwarn(str(msg.joint_position_angles_goal[i]))

            if msg.joint_position_angles_goal[i] < self.joint_positions[msg.joint_names_goal[i]]:
                cmd.effort = cmd.effort * -1

            rospy.logwarn('publishe ' + str(cmd.effort) + ' auf ' + str(cmd.name))
            self.arm_joint_pub.publish(cmd)

            #while abs(msg.joint_position_angles_goal[i] - self.joint_positions[msg.joint_names_goal[i]]) > self.goal_threshold:
            #    rospy.logwarn(str(abs(msg.joint_position_angles_goal[i] - self.joint_positions[msg.joint_names_goal[i]])))
            while True:
                if abs(msg.joint_position_angles_goal[i] - self.joint_positions[msg.joint_names_goal[i]]) > self.goal_threshold:
                    rospy.logwarn(str(abs(msg.joint_position_angles_goal[i] - self.joint_positions[msg.joint_names_goal[i]])))
                    continue
                else:
                    cmd.effort = 0.0
                    self.arm_joint_pub.publish(cmd)
                    time.sleep(.1)
                    self.arm_joint_pub.publish(cmd)
                    break
            rospy.logwarn('reached')

        self._result.joint_position_angles_result = [self.joint_positions[MOTOR_A], self.joint_positions[MOTOR_B], self.joint_positions[GRIPPER]]
        self._sas.set_succeeded(self._result)
        rospy.logwarn("ArmPositionServer succeeded")

    def joint_states_cb(self, data):
        if data.name[0] == MOTOR_A:
            self.joint_positions[MOTOR_A] = data.position[0]
            self.joint_0_vel = data.velocity[0]
            self.joint_0_eff = data.effort[0]
        if data.name[0] == MOTOR_B:
            self.joint_positions[MOTOR_B] = data.position[0]
            self.joint_1_vel = data.velocity[0]
            self.joint_1_eff = data.effort[0]
        if data.name[0] == GRIPPER:
            self.joint_positions[GRIPPER] = data.position[0]
            self.gripper_vel = data.velocity[0]
            self.gripper_eff = data.effort[0]

if __name__ == '__main__':
    rospy.init_node('arm_position')
    server = ArmPositionServer('ArmPosition')
    rospy.spin()
