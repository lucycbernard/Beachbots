#! /usr/bin/env python
# ==============================================================================
# title           :CameraInfo.py
# description     :Node to publish camera calibration info for apriltag detection
# author          :James Casella, Dennis Chavez Romero
# date            :2021-11-16
# version         :0.1
# notes           :
# python_version  :3.8
# ros parameters  : 
# ==============================================================================
import rospy
import cv2
from sensor_msgs.msg import CameraInfo

rospy.init_node('camera_info', anonymous=True)

pub = rospy.Publisher('/camera_rect/camera_info', CameraInfo, queue_size=10)
rate = rospy.Rate(60)

while not rospy.is_shutdown():
    q = CameraInfo()

    q.header.frame_id = 'usb_cam'
    q.height = 640
    q.width = 480

    q.D = [-0.005681561825020133, 0.1511321319570499, 0.0027610244181580507, -0.017597512679995944, 0.0]
    q.K = [728.6754810243508, 0.0, 277.61711936355425, 0.0, 720.4869766983572, 251.77583559891315, 0.0, 0.0, 1.0]
    q.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    q.P = [732.7319946289062, 0.0, 270.3580506368362, 0.0, 0.0, 733.597412109375, 251.97498305948102, 0.0, 0.0, 0.0, 1.0, 0.0]

    q.binning_x = 0
    q.binning_y = 0
    q.roi.x_offset = 0
    q.roi.y_offset = 0
    q.roi.height = 0
    q.roi.width = 0
    q.roi.do_rectify = False
    pub.publish(q)
    rate.sleep()
