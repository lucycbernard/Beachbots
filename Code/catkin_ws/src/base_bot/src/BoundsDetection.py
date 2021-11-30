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
from std_msgs.msg import Bool, String
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

        # Fetch parameters from ros parameter service
        self.refresh_rate = rospy.get_param("~Rate") # get the refresh rate of the detection
        self.ID_list = rospy.get_param("~ID List") # get all the AprilTag ID's in use
        self.upper_list = rospy.get_param("~Upper List") # get the upper limit of each AprilTag's position
        self.lower_list = rospy.get_param("~Lower List") # get the lower limit of each AprilTag's position

        rospy.delete_param("~Rate")
        rospy.delete_param("~ID List")
        rospy.delete_param("~Upper List")
        rospy.delete_param("~Lower List")

        # Create publisher for bounds data
        #self.pub = rospy.Publisher("/tag_bounds", Bool, queue_size=10)
        self.pub =  rospy.Publisher("/debug", String, queue_size=10)

        # Create subscriber to listen to AprilTag data
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.handleAprilTag)

        # rate to process tags
        self.rate = rospy.Rate(self.refresh_rate)

        # Parse strings to lists using built in split() function
        self.ID_list = self.ID_list.split()
        self.upper_list = self.upper_list.split()
        self.lower_list = self.lower_list.split()

        # Process lists into dictionaries
        self.pub_map = {} # mapping of AprilTag ID's to ros publishers
        self.upper_map = {} # mapping of AprilTag ID's to position upper bounds
        self.lower_map = {} # mapping of AprilTag ID's to position lower bounds
        for id_index in range(len(self.ID_list)):
            # for each ID in the ID_list, map the publisher, the upper bound, and the lower bound
            ID = int(self.ID_list[id_index]) # get the ID as an integer
            upper = float(self.upper_list[id_index]) # get the bound as a float
            lower = float(self.lower_list[id_index]) # get the bound as a float

            self.pub_map[ID] = rospy.Publisher("/tag_bounds/tag_" + str(ID), Bool, queue_size=10)
            self.upper_map[ID] = upper
            self.lower_map[ID] = lower

        # seep to let ros initialize
        rospy.sleep(2)

        # loop until ros is shutdown
        while not rospy.is_shutdown():
            self.rate.sleep()

    def handleAprilTag(self, ATArray):
        """
        Reads the detected AprilTags and handles any that are out of bounds
        :param ATArray         [apriltag_ros.msg.AprilTagDetectionArray] Array of detected AprilTags
        """

        for tag in ATArray.detections:
            xDist = tag.pose.pose.pose.position.x
            self.pub.publish(str(xDist))
            # self.pub.publish(str(tag.id[0]))
            if xDist < self.lower_map[tag.id[0]] or xDist > self.upper_map[tag.id[0]]:
                self.pub_map[tag.id[0]].publish(True)


if __name__ == "__main__":
    BD = BoundsDetection()
