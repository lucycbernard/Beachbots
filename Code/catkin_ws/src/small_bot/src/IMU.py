#!/usr/bin/env python
# ==============================================================================
# title           :AdafruitIMU.py
# description     :Reads data from the onboard IMU
# author          :James Casella, Dennis Chavez Romero, Spencer Gregg
# date            :2021-01-25
# version         :0.1
# notes           :
# python_version  :3.8
# ros parameters  : ID: [String] smallbot ID 
#                 : Rate: [Int] rate in Hz at which IMU data should be published
# ==============================================================================
import rospy
from std_msgs import Float32
import math
import adafruit_bno055
from adafruit_extended_bus import ExtendedI2C as I2C


class IMU:

    def __init__(self):
        """"
        Class constructor
        """

        # initialize ros node
        rospy.init_node("IMU", anonymous=True)

        # Fetch parameters from ros parameter service
        self.ID = rospy.get_param("~ID") # Get smallbot ID to name topics
        self.refresh_rate = rospy.get_param("~Rate") # Get the rate in Hz at which IMU data should be published

        # create publisher for heading data on topic "SmallBot_ID/Chassis/IMU/Heading"
        self.headingPublisher = rospy.Publisher("SmallBot_" + self.ID + "/Chassis/IMU/Heading", Float32, queue_size=10)

        #Create a rate object to control publishing speed
        self.rate = rospy.Rate(self.refresh_rate)

        # I2C data transmission
        i2c = I2C(1)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Temp variable
        self.lastAngle = 0

    def publish_imu_data(self):
        """
        Publish the IMU data until ros is shutdown
        """
        while not rospy.is_shutdown():
            self.headingPublisher.publish(self.get_yaw())
            self.rate.sleep()


    def get_yaw(self):
        """
        Calculates the smallbot's yaw in degrees by converting the IMU's quaternion values to euler angles
        Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        :return      [float]   The smallbot's yaw in degrees.
        """

        # Get the quaternions from the IMU
        x = self.sensor.quaternion[0]
        y = self.sensor.quaternion[1]
        z = self.sensor.quaternion[2]
        w = self.sensor.quaternion[3]

        # Check that there actual return values for every quaternion
        if not all(self.sensor.quaternion) or w == None or x == None or y == None or z == None:
            # If not, then return the last calculated yaw angle
            return self.lastAngle

        # Convert quaternions to euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        yaw_z = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        roll_x = math.atan2(t3, t4)

        # ZYX Convention
        yaw_z = math.degrees(yaw_z)
        pitch_y = math.degrees(pitch_y)
        roll_x = math.degrees(roll_x)

        # Map values from 0 to 360 ==> -180 to 180
        # When smallbot faces left, it will return a negative value
        # A positive value when facing right
        # A value of 0 when facing straight
        if yaw_z > 0:
            yaw_z -= 180
        elif yaw_z < 0:
            yaw_z += 180
        if yaw_z == None:
            return self.lastAngle
        else:
            self.lastAngle = yaw_z

        return yaw_z


# When run, call the IMU constructor to create the node
if __name__ == "__main__":
    IMU()