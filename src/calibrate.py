#!/usr/bin/python
print(80*"#")
#import roslib; roslib.load_manifest('nxt_controllers')
import rospy
print(40*"Oo")
import math
import thread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand, Contact
import time

MOTOR_A = "motor_1"
MOTOR_B = "motor_2"
JOINT_0_COMMAND = JointCommand()
JOINT_1_COMMAND = JointCommand()

class ArmCalibration:
    def __init__(self):
        self.initialized = False
        self.joint_0_angle = 0.0
        self.joint_1_angle = 0.0
        self.joint_0_vel = 0.0
        self.joint_1_vel = 0.0
        self.joint_0_eff = 0.0
        self.joint_1_eff = 0.0
        self.calibration_effort = 0.4
        self.joint_0_contact = False

        self.rate = rospy.Rate(2)

        JOINT_0_COMMAND.effort = 0.4
        JOINT_0_COMMAND.name = MOTOR_A
        JOINT_1_COMMAND.effort = -0.6
        JOINT_1_COMMAND.name = MOTOR_B


        # joint interaction
        self.pub = rospy.Publisher('nxt_1/joint_command', JointCommand, queue_size=10)
        rospy.Subscriber('nxt_1/joint_state', JointState, self.joint_states_cb)
        rospy.Subscriber('touch_sensor', Contact, self.touch_cb)

    def touch_cb(self, data):
        self.joint_0_contact = data.contact

    def joint_states_cb(self, data):
        print(str(data.name[0] + ": " + str(data.position[0])))

        if data.name[0] == MOTOR_A:
            self.joint_0_angle = data.position[0]
            self.joint_0_vel = data.velocity[0]
            self.joint_0_eff = data.effort[0]
        if data.name[0] == MOTOR_B:
            self.joint_1_angle = data.position[0]
            self.joint_1_vel = data.velocity[0]
            self.joint_1_eff = data.effort[0]

    def calibrate(self):
        joint_0_pos = -1
        joint_0_old = -2
        while not self.joint_0_contact:
            print("pos " + str(joint_0_pos))
            print("old " + str(joint_0_old))
            print("new " + str(self.joint_0_angle))
            print("abs " + str(abs(joint_0_pos - joint_0_old)))
            joint_0_old = joint_0_pos
            joint_0_pos = self.joint_0_angle
            JOINT_0_COMMAND.effort = 0.6
            self.pub.publish(JOINT_0_COMMAND)
            self.rate.sleep()
            time.sleep(.5)

        JOINT_0_COMMAND.effort = 0.0
        self.pub.publish(JOINT_0_COMMAND)
        print("!!!!!!CALIBRATED motor 1!!!!!!!")
        joint_1_pos = -1
        while joint_1_pos != self.joint_1_angle or abs(joint_1_pos) <= 1:
            print("##########" + str(self.joint_1_angle) + "##########")
            joint_1_pos = self.joint_1_angle
            JOINT_1_COMMAND.effort = -0.6
            self.pub.publish(JOINT_1_COMMAND)
            self.rate.sleep()
            time.sleep(.5)

        JOINT_1_COMMAND.effort = 0.0
        self.pub.publish(JOINT_1_COMMAND)
        print("!!!!!!CALIBRATED!!!!!!!")
        return



def main():
    rospy.init_node('arm_calibration')
    arm_calibration = ArmCalibration()
    arm_calibration.calibrate()
    rospy.spin()


if __name__ == '__main__':
    time.sleep(2)
    main()