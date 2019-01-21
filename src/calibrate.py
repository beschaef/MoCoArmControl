#!/usr/bin/python
#import roslib; roslib.load_manifest('nxt_controllers')
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand, Contact
import time

MOTOR_A = "motor_1"
MOTOR_B = "motor_2"
GRIPPER = "motor_3"
JOINT_0_COMMAND = JointCommand()
JOINT_1_COMMAND = JointCommand()
GRIPPER_COMMAND = JointCommand()

"""
This class provides all functions to calibrate the arm.
"""
class ArmCalibration:

    """
    Initialization function of the class.
    All class variables are created here and the ROS publisher and subscriber initialized.
    """
    def __init__(self):
        self.initialized = False

        self.joint_0_angle = 0.0
        self.joint_0_vel = 0.0
        self.joint_0_eff = 0.0

        self.joint_1_angle = 0.0
        self.joint_1_vel = 0.0
        self.joint_1_eff = 0.0

        self.gripper_angle = 0.0
        self.gripper_vel = 0.0
        self.gripper_eff = 0.0
        self.joint_0_contact = False

        self.rate = rospy.Rate(2)

        JOINT_0_COMMAND.effort = 0.4
        JOINT_0_COMMAND.name = MOTOR_A
        JOINT_1_COMMAND.effort = -0.6
        JOINT_1_COMMAND.name = MOTOR_B
        GRIPPER_COMMAND.effort = -0.6
        GRIPPER_COMMAND.name = GRIPPER

        self.pub = rospy.Publisher('nxt_1/joint_command', JointCommand, queue_size=10)
        rospy.Subscriber('nxt_1/joint_state', JointState, self.joint_states_cb)
        rospy.Subscriber('touch_sensor', Contact, self.touch_cb)
        rospy.logwarn("Initialized calibration node")

    """
    Callback function for the touch sensor.
    Here, the value of the sensor is read out and stored in a class variable.
    
    Parameters
    ----------
    data : nxt_msgs.Contact
        Value of the touch sensor
    """
    def touch_cb(self, data):
        self.joint_0_contact = data.contact

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
            self.joint_0_angle = data.position[0]
            self.joint_0_vel = data.velocity[0]
            self.joint_0_eff = data.effort[0]
            rospy.logwarn(data.position[0])
        if data.name[0] == MOTOR_B:
            self.joint_1_angle = data.position[0]
            self.joint_1_vel = data.velocity[0]
            self.joint_1_eff = data.effort[0]
        if data.name[0] == GRIPPER:
            self.gripper_angle = data.position[0]
            self.gripper_vel = data.velocity[0]
            self.gripper_eff = data.effort[0]

    """
    Calibrates each joint.
    First the lower, then the upper and finally the gripper.
    """
    def calibrate(self):

        self.calibrate_joint_0()

        self.calibrate_joint_1()

        self.calibrate_gripper()

        return

    """
    Calibrates the lower joint.
    As long as the touch sensor has no contact, the motor continues to rotate.
    """
    def calibrate_joint_0(self):
        while not self.joint_0_contact:
            JOINT_0_COMMAND.effort = 0.65
            self.pub.publish(JOINT_0_COMMAND)
            self.rate.sleep()
            time.sleep(.5)

        JOINT_0_COMMAND.effort = 0.0
        self.pub.publish(JOINT_0_COMMAND)
        rospy.logwarn("calibrated joint 0")

    """
    
    """
    def calibrate_joint_1(self):
        joint_1_pos = -1

        while abs(joint_1_pos - self.joint_1_angle) > 0.7 or abs(joint_1_pos) <= 1:
            joint_1_pos = self.joint_1_angle
            JOINT_1_COMMAND.effort = -0.6
            self.pub.publish(JOINT_1_COMMAND)
            self.rate.sleep()
            time.sleep(.5)

        JOINT_1_COMMAND.effort = 0.0
        self.pub.publish(JOINT_1_COMMAND)
        rospy.logwarn("calibrated joint 1")

    def calibrate_gripper(self):
        gripper_pos = -1
        gripper_old = -2

        GRIPPER_COMMAND.effort = 0.70
        self.pub.publish(GRIPPER_COMMAND)
        self.rate.sleep()
        time.sleep(.2)
        self.pub.publish(GRIPPER_COMMAND)
        self.rate.sleep()
        time.sleep(.2)
        while abs(gripper_pos - gripper_old) > 0.8 or abs(gripper_pos) <= 1:
            rospy.logwarn(
                "old: " + str(gripper_old) + "\npos: " + str(gripper_pos) + "\nang: " + str(self.gripper_angle))
            gripper_old = gripper_pos
            gripper_pos = self.gripper_angle
            GRIPPER_COMMAND.effort = 0.6
            self.pub.publish(GRIPPER_COMMAND)
            self.rate.sleep()
            time.sleep(.5)

        GRIPPER_COMMAND.effort = 0.0
        self.pub.publish(GRIPPER_COMMAND)
        rospy.logwarn("calibrated gripper")



def main():
    rospy.init_node('arm_calibration')
    arm_calibration = ArmCalibration()
    #arm_calibration.calibrate()
    rospy.spin()


if __name__ == '__main__':
    time.sleep(2)
    main()
