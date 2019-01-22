#!/usr/bin/env python

import roslib;
import rospy
from actionlib import *
from actionlib_msgs.msg import *
from nxt_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from nxt_action_msgs.msg import ArmPositionFeedback, ArmPositionResult, ArmPositionAction

import time
import unittest

MOTOR_A = "motor_1"
MOTOR_B = "motor_2"
GRIPPER = "motor_3"


"""
The action server provides the control for the arm.
Only the target angles have to be passed to this.
The action server then automatically adjusts the target angles.
"""
class ArmPositionServer:
    _feedback = ArmPositionFeedback()
    _result = ArmPositionResult()

    """
    Initialization function of the ActionServer.
    Initializes the class variables and the required subscribers and publishers.
    """
    def __init__(self, name, is_test=False):

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

        # Since the ROS sections without the arm are not readily testable, these were not initialized for the test cases
        if not is_test:
            self._sas = SimpleActionServer(name, ArmPositionAction, execute_cb=self.execute_cb, auto_start=False)
            self.arm_joint_pub = rospy.Publisher("nxt_1/joint_command", JointCommand, queue_size=10)
            self.arm_state_sub = rospy.Subscriber("nxt_1/joint_state", JointState, self.joint_states_cb)
            self._sas.start()

    """
    The function is given target angle of the various engines.
    Subsequently, the various joints or motors are driven from bottom to top 
    (lower joint, upper joint, Gripper) to the target position. Since the motors 
    can only be controlled via the effort and the angles can not be read out 
    exactly, the target angles are approached up to a threshold.
    
    Parameters
    ----------
    msg : ArmPositionAction
        includes the names of the joints and the corresponding angles.
    """
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

    """
    Callback function for the servomotors.
    The values of the motors are stored in the corresponding class variables.

    Parameters
    ----------
    data : sensor_msgs.JointState
        Information of the Motor State. Data includes position, velocity and effort.
    """
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

"""
Testclass for ArmPositionServer Class
To test the code run 'python -m unittest calibrate.TestArmPositionServer' in src folder.
"""
class TestArmPositionServer(unittest.TestCase):

    """
    Creates a new Object and all needed Variables
    """
    def setUp(self):
        self.arm_pos_server = ArmPositionServer("",True)
        self.test_joint_states = JointState()
        self.test_joint_states.position = [13.37]
        self.test_joint_states.velocity = [13.37]
        self.test_joint_states.effort = [1.0]

    """
    test callback function when lower motor is given
    """
    def test_joint_states_cb_wrong_values(self):
        self.test_joint_states.name = ["motor_42"]
        self.test_joint_states.position = [13.37]
        self.test_joint_states.velocity = [13.37]
        self.test_joint_states.effort = [1.0]
        self.arm_pos_server.joint_states_cb(self.test_joint_states)
        self.assertNotAlmostEqual(13.37, self.arm_pos_server.joint_positions[MOTOR_A], places=7, msg="Angle: different to high",
                                  delta=None)
        self.assertNotAlmostEqual(13.37, self.arm_pos_server.joint_0_vel, places=7, msg="Velocity: different to high",
                                  delta=None)
        self.assertNotAlmostEqual(1.0, self.arm_pos_server.joint_0_eff, places=7, msg="Effort: different to high", delta=None)
        self.assertNotAlmostEqual(13.37, self.arm_pos_server.joint_positions[MOTOR_B], places=7, msg="Angle: different to high",
                                  delta=None)
        self.assertNotAlmostEqual(13.37, self.arm_pos_server.joint_1_vel, places=7, msg="Velocity: different to high",
                                  delta=None)
        self.assertNotAlmostEqual(1.0, self.arm_pos_server.joint_1_eff, places=7, msg="Effort: different to high", delta=None)
        self.assertNotAlmostEqual(13.37, self.arm_pos_server.joint_positions[GRIPPER], places=7, msg="Angle: different to high",
                                  delta=None)
        self.assertNotAlmostEqual(13.37, self.arm_pos_server.gripper_vel, places=7, msg="Velocity: different to high",
                                  delta=None)
        self.assertNotAlmostEqual(1.0, self.arm_pos_server.gripper_eff, places=7, msg="Effort: different to high", delta=None)

    """
    test callback function when lower motor is given
    """
    def test_joint_states_cb_motor_0(self):
        self.test_joint_states.name = ["motor_1"]
        self.arm_pos_server.joint_states_cb(self.test_joint_states)
        self.assertAlmostEqual(13.37, self.arm_pos_server.joint_positions[MOTOR_A], places=7, msg="Angle: different to high", delta=None)
        self.assertAlmostEqual(13.37, self.arm_pos_server.joint_0_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertAlmostEqual(1.0, self.arm_pos_server.joint_0_eff, places=7, msg="Effort: different to high", delta=None)

    """
    test callback function when upper motor is given
    """
    def test_joint_states_cb_motor_1(self):
        self.test_joint_states.name = ["motor_2"]
        self.arm_pos_server.joint_states_cb(self.test_joint_states)
        self.assertAlmostEqual(13.37, self.arm_pos_server.joint_positions[MOTOR_B], places=7, msg="Angle: different to high", delta=None)
        self.assertAlmostEqual(13.37, self.arm_pos_server.joint_1_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertAlmostEqual(1.0, self.arm_pos_server.joint_1_eff, places=7, msg="Effort: different to high", delta=None)

    """
    test callback function when gripper motor is given
    """
    def test_joint_states_cb_gripper(self):
        self.test_joint_states.name = ["motor_3"]
        self.arm_pos_server.joint_states_cb(self.test_joint_states)
        self.assertAlmostEqual(13.37, self.arm_pos_server.joint_positions[GRIPPER], places=7, msg="Angle: different to high", delta=None)
        self.assertAlmostEqual(13.37, self.arm_pos_server.gripper_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertAlmostEqual(1.0, self.arm_pos_server.gripper_eff, places=7, msg="Effort: different to high", delta=None)


if __name__ == '__main__':
    rospy.init_node('arm_position')
    server = ArmPositionServer('ArmPosition')
    rospy.spin()
