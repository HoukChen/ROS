#!/usr/bin/env python

# This script subscribes to the rgb and depth image topic of the ROS camera, 
# then transforms the data in the topics to be images via cv_brige

import roslib
import sys
import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback_rgb(data):
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(data)
  cv2.imwrite(str(time.time())+'rgb.png', cv_image)
  #cv2.imshow('image window', cv_image)
  #cv2.waitKey(0)

def callback_depth(data):
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data)
	cv2.imwrite(str(time.time())+'dep.png', cv_image)


def main():
  rospy.init_node('Subscribe_Image_Node')
  rospy.Subscriber('/camera/rgb/image_raw', Image, callback=callback_rgb, queue_size=10)
  rospy.Subscriber('/camera/depth/image_raw', Image, callback=callback_depth, queue_size=10)
  rospy.spin()

if __name__ == '__main__':
  main()