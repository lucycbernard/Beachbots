#!/usr/bin/env python
# ==============================================================================
# title           :Chassis.py
# description     :Node for small_bot drivetrain
# author          :James Casella, Dennis Chavez Romero, Spencer Gregg, Matthew Langkamp, Yossef Naim
# date            :2021-11-03
# version         :0.1
# notes           :
# python_version  :3.8
# ros parameters  :
# ==============================================================================

import rospy
from small_bot.msg import DriveHeading
from std_msgs.msg import Float32, Int16, String, Bool
import RPi.GPIO as GPIO
import sys


class Chassis:

    def __init__(self):
        """
        Class constructor
        """
        # initialize ros node
        rospy.init_node("Chassis", anonymous=True)

        # Fetch parameters from ros parameter service
        self.ID = rospy.get_param("~ID")  # Get smallbot ID to name topics
        self.Tag_ID = rospy.get_param("~Tag_ID")  # Get the AprilTag number used on this SmallBot
        self.RPWMF = rospy.get_param("~RPWMF")  # RIGHT PWM FORWARDS
        self.RPWMB = rospy.get_param("~RPWMB")  # RIGHT PWM BACKWARDS
        self.LPWMF = rospy.get_param("~LPWMF")  # LEFT PWM FORWARDS
        self.LPWMB = rospy.get_param("~LPWMB")  # LEFT PWM BACKWARDS
        self.Turn_Threshold = rospy.get_param("~Turn_Threshold")  # number of consecutive values to initiate turn

        # Clean parameter service
        rospy.delete_param("~ID")
        rospy.delete_param("~Tag_ID")
        rospy.delete_param("~RPWMF")
        rospy.delete_param("~RPWMB")
        rospy.delete_param("~LPWMF")
        rospy.delete_param("~LPWMB")
        rospy.delete_param("~Turn_Threshold")

        # create publisher for heading data on topic "SmallBot_ID/Chassis/IMU/Heading"
        # self.headingPublisher = rospy.Publisher("SmallBot_" + self.ID + "/Chassis/IMU/Heading", Float32, queue_size=10)

        # Disable warnings
        GPIO.setwarnings(False)

        # Set pin numbering system
        GPIO.setmode(GPIO.BOARD)

        # Setup pins as OUT for output
        GPIO.setup(self.RPWMF, GPIO.OUT)
        GPIO.setup(self.RPWMB, GPIO.OUT)
        GPIO.setup(self.LPWMF, GPIO.OUT)
        GPIO.setup(self.LPWMB, GPIO.OUT)

        # Create PWM instance with frequency
        self.pi_rpwmf = GPIO.PWM(self.RPWMF, 1000)
        self.pi_rpwmb = GPIO.PWM(self.RPWMB, 1000)
        self.pi_lpwmf = GPIO.PWM(self.LPWMF, 1000)
        self.pi_lpwmb = GPIO.PWM(self.LPWMB, 1000)

        # Start PWM of required Duty Cycle
        self.pi_rpwmf.start(0)
        self.pi_rpwmb.start(0)
        self.pi_lpwmf.start(0)
        self.pi_lpwmb.start(0)

        # counters for handling bounds detection
        self.leftCount = 0  # left count is number of -1's
        self.rightCount = 0  # right count is number of 1's
        self.lastTurnValue = 2  # last seen value -1/0/1 ; 2 is default value for nothing seen yet

        # degubber
        self.helppub = rospy.Publisher('/chatter', String, queue_size=10)

        # tracker for last turn executed
        self.lastTurnPerformed = 0

        # PID constants
        self.k_p = 1
        self.k_i = 0
        self.k_d = 0
        self.PID_index = 1
        self.PID_array = [0]
        self.PID_array_len = 30

        rospy.sleep(1)

    def drive(self, right_speed, left_speed):
        """
        Moves the smallbot chassis based on right and left wheel efforts
        :param right_speed     [int]  The wheel efforts for the right side of drivetrain.
        :param left_speed      [int]  The wheel efforts for the left side of drivetrain.
        """
        # Define behavior for all possible effort combinations

        # Limit motor effort to between -100 and 100.
        if (abs(left_speed) > 100):
            left_speed = (left_speed / abs(left_speed)) * 100
        if (abs(right_speed) > 100):
            right_speed = (right_speed / abs(right_speed)) * 100

        # Straight forward
        if (right_speed > 0) and (left_speed > 0):
            self.pi_rpwmf.ChangeDutyCycle(right_speed)
            self.pi_lpwmf.ChangeDutyCycle(left_speed)
            self.pi_rpwmb.ChangeDutyCycle(0)
            self.pi_lpwmb.ChangeDutyCycle(0)

        # Straight backwards
        elif (right_speed < 0) and (left_speed < 0):
            self.pi_rpwmb.ChangeDutyCycle(abs(right_speed))
            self.pi_lpwmb.ChangeDutyCycle(abs(left_speed))
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_lpwmf.ChangeDutyCycle(0)

        # Point turn left
        elif (right_speed > 0) and (left_speed < 0):
            self.pi_rpwmf.ChangeDutyCycle(right_speed)
            self.pi_lpwmb.ChangeDutyCycle(abs(left_speed))
            self.pi_rpwmb.ChangeDutyCycle(0)
            self.pi_lpwmf.ChangeDutyCycle(0)

        # Point turn right
        elif (right_speed < 0) and (left_speed > 0):
            self.pi_rpwmb.ChangeDutyCycle(abs(right_speed))
            self.pi_lpwmf.ChangeDutyCycle(left_speed)
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_lpwmb.ChangeDutyCycle(0)

        # Forwards swing turn right
        elif (right_speed == 0) and (left_speed > 0):
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_rpwmb.ChangeDutyCycle(0)
            self.pi_lpwmf.ChangeDutyCycle(left_speed)
            self.pi_lpwmb.ChangeDutyCycle(0)

        # Backwards swing turn right
        elif (right_speed == 0) and (left_speed < 0):
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_rpwmb.ChangeDutyCycle(0)
            self.pi_lpwmb.ChangeDutyCycle(abs(left_speed))
            self.pi_lpwmf.ChangeDutyCycle(0)

        # Forwards swing turn left
        elif (right_speed > 0) and (left_speed == 0):  # for swing turn left
            self.pi_rpwmf.ChangeDutyCycle(right_speed)
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_lpwmf.ChangeDutyCycle(0)
            self.pi_lpwmb.ChangeDutyCycle(0)

        # Backwards swing turn left
        elif (right_speed < 0) and (left_speed == 0):  # for swing turn left
            self.pi_rpwmb.ChangeDutyCycle(abs(right_speed))
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_lpwmf.ChangeDutyCycle(0)
            self.pi_lpwmb.ChangeDutyCycle(0)

        # Full stop
        elif right_speed == 0 and left_speed == 0:
            self.pi_lpwmf.ChangeDutyCycle(0)
            self.pi_lpwmb.ChangeDutyCycle(0)
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_rpwmb.ChangeDutyCycle(0)

if __name__ == "__main__":
    chassis = Chassis()

    #wait 5 seconds then begin driving procedure
    #rospy.sleep(5)

    #chassis.startMotion()
    while not rospy.is_shutdown():
        #        chassis.drive(10,10)
        chassis.drive(20,20)
#        print("no end of file here")
