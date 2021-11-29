#!/usr/bin/env python
# ==============================================================================
# title           :BoundsDetection.py
# description     :Listens to TF data from AprilTag detection to see if a tag is out of bounds
# author          :James Casella
# date            :2021-11-29
# version         :0.1
# notes           :
# python_version  :3.8
# ros parameters  : 
# ==============================================================================
import rospy
from std_msgs.msg import Bool
import tf
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import math



class BoundsDetection:

    def __init__(self):
        """"
        Class constructor
        """

        # initialize ros node
        rospy.init_node("BoundsDetection", anonymous=True)

        # get bounds from parameter service (for now hard code for a single tag)
        #  lower and upper are the bounds in meters of the tag
        self.lower = 0.4
        self.upper = 0.6

        # Create publisher for bounds data
        self.pub = rospy.Publisher("/tag_bounds", Bool, queue_size=10)

        # Create subscriber to listen to AprilTag data
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.handleAprilTag)

        # rate to process tags
        Rate = 0.15

        # seep to let ros initialize
        rospy.sleep(2)

        # loop until ros is shutdown
        while not rospy.is_shutdown():
            rospy.sleep(Rate)

    def handleAprilTag(self, ATArray):
        """
        Reads the detected AprilTags and handles any that are out of bounds
        :param ATArray         [apriltag_ros.msg.AprilTagDetectionArray] Array of detected AprilTags
        """

        for tag in ATArray.detections:
            zDist = tag.pose.pose.pose.position.z
            print(zDist)
            if zDist < self.lower or zDist > self.upper:
                self.pub.publish(True)


if __name__ == "__main__":
    BD = BoundsDetection()
