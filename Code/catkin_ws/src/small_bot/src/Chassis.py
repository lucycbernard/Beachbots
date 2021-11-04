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
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from AdafruitIMU import AdafruitIMU
import sys


class Chassis:

    def __init__(self):
        """
        Class constructor
        """
        # initialize ros node
        rospy.init_node("IMU", anonymous=True)

        # Fetch parameters from ros parameter service
        self.ID = rospy.get_param("~ID") # Get smallbot ID to name topics
        rospy.delete_param("~ID") # Get smallbot ID to name topics

        # create publisher for heading data on topic "SmallBot_ID/Chassis/IMU/Heading"
        # self.headingPublisher = rospy.Publisher("SmallBot_" + self.ID + "/Chassis/IMU/Heading", Float32, queue_size=10)
        
        # Subscribe to 
        rospy.Subscriber("SmallBot_" + self.ID + "/Chassis/IMU/Heading", Float32, self.updateHeading)
        self.heading = 0

        # Set PWM pins for motors
        self.RPWMF = rospy.get_param("~RPWMF")  # RIGHT PWM FORWARDS
        self.RPWMB = rospy.get_param("~RPWMB")  # RIGHT PWM BACKWARDS
        self.LPWMF = rospy.get_param("~LPWMF")  # LEFT PWM FORWARDS
        self.LPWMB = rospy.get_param("~LPWMB")  # LEFT PWM BACKWARDS

        rospy.delete_param("~RPWMF")  # RIGHT PWM FORWARDS
        rospy.delete_param("~RPWMB")  # RIGHT PWM BACKWARDS
        rospy.delete_param("~LPWMF")  # LEFT PWM FORWARDS
        rospy.delete_param("~LPWMB")  # LEFT PWM BACKWARDS
        

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

    def reset_heading(self):
        """
        Resets the IMU heading by declaring a new instance of the IMU
        """
        self.IMU = AdafruitIMU()

    def drive(self, right_speed, left_speed):
        """
        Moves the smallbot chassis based on right and left wheel efforts
        :param right_speed     [int]  The wheel efforts for the right side of drivetrain.
        :param left_speed      [int]  The wheel efforts for the left side of drivetrain.
        """
        # Define behavior for all possible effort combinations

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

    def point_turn_IMU(self, wanted_angle, speed):
        """
        Moves the smallbot chassis to a desired angle relative from where it currently is by turning about its center
        :param wanted_angle    [int]  The desired angle for the chassis to turn to in degrees.
        :param speed           [int]  The wheel effort to apply to the chassis while performing the turn.
        """

        # Keep turning while the current IMU yaw angle is not within our defined threshold
        while (self.IMU.get_yaw() > wanted_angle + Constants.DEG_THRESHOLD or self.IMU.get_yaw()
               < wanted_angle - Constants.DEG_THRESHOLD):

            # Turn right for a positive angle
            if wanted_angle > 0:
                self.drive(-speed, speed)
            # Turn left for a negative angle
            elif wanted_angle < 0:
                self.drive(speed, -speed)
            # Handling for when wanted angle is 0 (aka straight heading)
            elif self.IMU.get_yaw() > 0 and wanted_angle == 0:
                self.drive(speed, -speed)
            elif self.IMU.get_yaw() < 0 and wanted_angle == 0:
                self.drive(-speed, speed)

        # Stop after completing point turn
        self.drive(0, 0)

    def point_turn_basebot(self, wanted_angle, speed):
        """
        Moves the smallbot chassis to a desired angle relative from where it currently is by turning about its center
        without the use of any loops
        :param wanted_angle    [int]  The desired angle for the chassis to turn to in degrees.
        :param speed           [int]  The wheel effort to apply to the chassis while performing the turn.
        :return                [bool] Boolean indicating whether or not the wanted angle has been achieved.
        """

        # Check if the current IMU yaw angle is within our defined threshold
        if (self.IMU.get_yaw() > wanted_angle + Constants.DEG_THRESHOLD or
                self.IMU.get_yaw() < wanted_angle - Constants.DEG_THRESHOLD):

            # Turn right for a positive angle
            if wanted_angle > 0:
                self.drive(-speed, speed)
            # Turn left for a negative angle
            elif wanted_angle < 0:
                self.drive(speed, -speed)
            # Handling for when wanted angle is 0 (aka straight heading)
            elif self.IMU.get_yaw() > 0 and wanted_angle == 0:
                self.drive(speed, -speed)
            elif self.IMU.get_yaw() < 0 and wanted_angle == 0:
                self.drive(-speed, speed)

        else:
            # Stop after completing point turn
            self.drive(0, 0)

            # Return true if desired angle has been achieved
            return True

        # Return false if desired angle has not been achieved
        return False

    def drive_straight_IMU(self, straight_speed, curr_angle):
        """
        Allows the drivetrain to drive straight while maintaining a desired heading by using a simple P controller
        :param straight_speed     [int]  The wheel effort to apply while driving straight.
        :param curr_angle         [int]  The desired heading to maintain while driving straight.
        """

        # Set the target as the desired angle
        target = curr_angle

        # Capture current yaw angle
        absolute = self.IMU.get_yaw()

        # Adjust wheel efforts accordingly
        left_speed = straight_speed - (absolute - target)
        right_speed = straight_speed + (absolute - target)

        # Write calculated wheel efforts to chassis
        self.drive(right_speed, left_speed)

    def updateHeading(self,data):
        """
        Updates the heading stored within the class
        :param data         [std_msgs.msg.Float32] current robot heading
        """
        self.heading = data.data

    def spinDeg(self,deg):
        """
        Spins the robot to the given heading in degrees
        :param deg  [float] desired heading in degrees
        """
        while not rospy.is_shutdown():
            e = self.heading - deg
            self.drive(e,-e)



if __name__ == "__main__":
    chassis = Chassis()
    chassis.spinDeg(90)