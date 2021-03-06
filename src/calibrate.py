#!/usr/bin/python
#import roslib; roslib.load_manifest('nxt_controllers')
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand, Contact
import time
import unittest
import multiprocessing
import threading

MOTOR_A = "motor_1"
MOTOR_B = "motor_2"
GRIPPER = "motor_3"
JOINT_0_COMMAND = JointCommand()
JOINT_1_COMMAND = JointCommand()
GRIPPER_COMMAND = JointCommand()

"""
Since during the test cases the "real" system is not running,
and thus also ROS is not running, all ROS relevant data be mocked.
For this purpose, the function pointers of the public and sleep functions 
are overwritten with "empty" functions
"""
class mock_ros:

    def publish(self, data):
        pass

    def sleep(self):
        pass

"""
This class provides all functions to calibrate the arm.
"""
class ArmCalibration:

    """
    Initialization function of the class.
    All class variables are created here and the ROS publisher and subscriber initialized.
    """
    def __init__(self, is_test=False):
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


        JOINT_0_COMMAND.effort = 0.4
        JOINT_0_COMMAND.name = MOTOR_A
        JOINT_1_COMMAND.effort = -0.6
        JOINT_1_COMMAND.name = MOTOR_B
        GRIPPER_COMMAND.effort = -0.6
        GRIPPER_COMMAND.name = GRIPPER

        # Since the ROS sections without the arm are not readily testable,
        #  these were not initialized for the test cases.
        if not is_test:
            self.rate = rospy.Rate(2)
            self.pub = rospy.Publisher('nxt_1/joint_command', JointCommand, queue_size=10)
            rospy.Subscriber('nxt_1/joint_state', JointState, self.joint_states_cb)
            rospy.Subscriber('touch_sensor', Contact, self.touch_cb)
            rospy.logwarn("Initialized calibration node")
        else:
            self.pub = mock_ros()
            self.rate = mock_ros()

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
            #rospy.logwarn(data.position[0])
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
        return True

    """
    Calibrates the upper joint.
    Since no sensor was installed on the upper joint, the upper arm is partially driven to the stop.
    Thus it can be checked via the position difference between two measurements whether the joint has
    reached the stop and is thus calibrated.
    In addition, the motor is then stopped to not unnecessarily burden both the engine and the arm.
    """
    def calibrate_joint_1(self):
        joint_1_pos = -1
        while abs(joint_1_pos - self.joint_1_angle) > 0.7 or abs(joint_1_pos) <= 1:
            rospy.logwarn(abs(joint_1_pos - self.joint_1_angle))
            joint_1_pos = self.joint_1_angle
            JOINT_1_COMMAND.effort = -0.65
            self.pub.publish(JOINT_1_COMMAND)
            self.rate.sleep()
            time.sleep(.5)

        JOINT_1_COMMAND.effort = 0.0
        self.pub.publish(JOINT_1_COMMAND)
        time.sleep(.5)
        self.pub.publish(JOINT_1_COMMAND)
        rospy.logwarn("calibrated joint 1")
        return True

    """
    Calibrates the gripper.
    Since no sensor was installed on the gripper, the gripper is partially driven to the stop.
    Thus it can be checked via the position difference between two measurements whether the gripper has
    reached the stop and is thus calibrated.
    Since the worm gear on the gripper has a high resistance, the motor must first be driven with a higher force.
    In addition, the motor is then stopped to not unnecessarily burden both the engine and the gripper.
    """
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
        while abs(gripper_pos - gripper_old) > 0.6 or abs(gripper_pos) <= 1:
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
        return True


"""
Creates a new object 'ArmCalibration'
"""
def main():
    rospy.init_node('arm_calibration')
    arm_calibration = ArmCalibration()
    arm_calibration.calibrate()
    #rospy.spin()
    exit()

"""
Testclass for ArmCalibration Class
To test the code run 'python -m unittest calibrate.TestArmCalibration' in src folder.
"""
class TestArmCalibration(unittest.TestCase):

    """
    Creates a new Object and all needed Variables
    """
    def setUp(self):
        self.cal = ArmCalibration(True)
        self.contact = Contact()
        self.test_joint_states = JointState()
        self.contact.contact = True
        self.test_joint_states.position = [13.37]
        self.test_joint_states.velocity = [13.37]
        self.test_joint_states.effort = [1.0]

        self.stop_feeder = False

    """
    Since real sensors can not always provide exactly the same 
    values and the ROS subscriber can not ensure that no wrong 
    data is being fed in from outside, the callback functions 
    are fed with data here. Thus, exactly the same values can 
    be repeatedly tested.
    In this case, the touch sensor is simulated. Every 0.1 
    second (10 hertz) new values are published. After 20 values
    (or 2 seconds), the value is set to 'True'.
    """
    def feed_touch_sensor(self):
        i = 0
        while i < 50:
            if i > 20:
                self.cal.joint_0_contact = True
            i += 1
            time.sleep(.1)

    """
    As in the function 'feed_touch_sensor' values are simulated here 
    as well. Here, the calibration drive is to be simulated. As soon 
    as the arm is in the minimum position, the engine turns slower 
    and slower until it finally stops due to the effort. To simulate 
    this, every 0.2 seconds (5 hertz) new values are published, which 
    grow more and more slowly (i = (i + 1) * 0.9).
    """
    def feed_motor_values(self):
        self.stop_feeder = False
        i = 0.0
        decrease = 0.95
        while not self.stop_feeder:
            self.cal.joint_0_angle = i
            self.cal.joint_1_angle = i
            self.cal.gripper_angle = i
            i = (i + 1) * decrease
            time.sleep(.2)

    """
    tests calibrating the lower joint.
    """
    def test_calibrate_joint_0(self):
        threads = []
        t = threading.Thread(name='my_service', target=self.feed_touch_sensor)
        threads.append(t)

        t.start()
        self.assertEquals(True, self.cal.calibrate_joint_0(), "Callback not working")
        t.join()


    """
    tests calibrating the upper joint.
    """
    def test_calibrate_joint_1(self):
        threads = []
        t = threading.Thread(target=self.feed_motor_values)
        threads.append(t)
        t.start()
        self.assertEquals(True, self.cal.calibrate_joint_1(), "Callback not working")
        self.stop_feeder = True

    """
    tests calibrating the gripper.
    """
    def test_calibrate_gripper(self):
        threads = []
        t = threading.Thread(target=self.feed_motor_values)
        threads.append(t)
        t.start()
        self.assertEquals(True, self.cal.calibrate_gripper(), "Callback not working")
        self.stop_feeder = True

    """
    test callback function of touch sensor
    """
    def test_touch_cb(self):
        self.cal.touch_cb(self.contact)
        self.assertEquals(True, self.cal.joint_0_contact, "Callback not working")

    """
    test callback function when lower motor is given
    """
    def test_joint_states_cb_wrong_values(self):
        self.test_joint_states.name = ["motor_42"]
        self.test_joint_states.position = [13.37]
        self.test_joint_states.velocity = [13.37]
        self.test_joint_states.effort = [1.0]
        self.cal.joint_states_cb(self.test_joint_states)
        self.assertNotAlmostEqual(13.37, self.cal.joint_0_angle, places=7, msg="Angle: different to high", delta=None)
        self.assertNotAlmostEqual(13.37, self.cal.joint_0_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertNotAlmostEqual(1.0, self.cal.joint_0_eff, places=7, msg="Effort: different to high", delta=None)
        self.assertNotAlmostEqual(13.37, self.cal.joint_1_angle, places=7, msg="Angle: different to high", delta=None)
        self.assertNotAlmostEqual(13.37, self.cal.joint_1_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertNotAlmostEqual(1.0, self.cal.joint_1_eff, places=7, msg="Effort: different to high", delta=None)
        self.assertNotAlmostEqual(13.37, self.cal.gripper_angle, places=7, msg="Angle: different to high", delta=None)
        self.assertNotAlmostEqual(13.37, self.cal.gripper_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertNotAlmostEqual(1.0, self.cal.gripper_eff, places=7, msg="Effort: different to high", delta=None)

    """
    test callback function when lower motor is given
    """
    def test_joint_states_cb_motor_0(self):
        self.test_joint_states.name = ["motor_1"]
        self.cal.joint_states_cb(self.test_joint_states)
        self.assertAlmostEqual(13.37, self.cal.joint_0_angle, places=7, msg="Angle: different to high", delta=None)
        self.assertAlmostEqual(13.37, self.cal.joint_0_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertAlmostEqual(1.0, self.cal.joint_0_eff, places=7, msg="Effort: different to high", delta=None)

    """
    test callback function when upper motor is given
    """
    def test_joint_states_cb_motor_1(self):
        self.test_joint_states.name = ["motor_2"]
        self.cal.joint_states_cb(self.test_joint_states)
        self.assertAlmostEqual(13.37, self.cal.joint_1_angle, places=7, msg="Angle: different to high", delta=None)
        self.assertAlmostEqual(13.37, self.cal.joint_1_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertAlmostEqual(1.0, self.cal.joint_1_eff, places=7, msg="Effort: different to high", delta=None)

    """
    test callback function when gripper motor is given
    """
    def test_joint_states_cb_gripper(self):
        self.test_joint_states.name = ["motor_3"]
        self.cal.joint_states_cb(self.test_joint_states)
        self.assertAlmostEqual(13.37, self.cal.gripper_angle, places=7, msg="Angle: different to high", delta=None)
        self.assertAlmostEqual(13.37, self.cal.gripper_vel, places=7, msg="Velocity: different to high", delta=None)
        self.assertAlmostEqual(1.0, self.cal.gripper_eff, places=7, msg="Effort: different to high", delta=None)

"""
To ensure that the arm has been fully initialized, wait 2 seconds before starting.
"""
if __name__ == '__main__':
    time.sleep(2)
    main()
