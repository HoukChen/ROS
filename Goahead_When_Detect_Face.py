#!/usr/bin/env python

# This script gives a fixed velocity to the robot whenever a face is captured

import roslib
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callable()	
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(data, "bgr8")


  face_cascade = cv2.CascadeClassifier('../haarcascades/haarcascade_frontalface_default.xml')
  body_cascade = cv2.CascadeClassifier('../haarcascades/haarcascade_mcs_upperbody.xml')
  eye_cascade = cv2.CascadeClassifier('../haarcascades/haarcascade_eye.xml')

  gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

  faces = face_cascade.detectMultiScale(gray, 1.3, 5)
  bodys = body_cascade.detectMultiScale(gray, 1.3, 5)
  eyes = eye_cascade.detectMultiScale(gray, 1.3, 5)

  if (len(faces) or len(bodys) or len(eyes)):
  	vel_msg = Twist()
  	vel_msg.linear.x = 0
  	vel_msg.linear.y = 0
  	vel_msg.linear.z = 0
  	vel_msg.angular.z = 0
  	vel_msg.angular.z = 0
  	vel_msg.angular.z = 1
  	pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
  	pub.publish(vel_msg)

  #cv2.imshow('image window', cv_image)
  #cv2.waitKey(0)

def main():
  rospy.init_node('Subscribe_Image_Node')
  rospy.Subscriber('/camera/color/image_raw', Image, callback=callback, queue_size=10)
  rospy.spin()

if __name__ == '__main__':
  main()
