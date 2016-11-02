#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Image

def Publish(lx,ly,lz,ax,ay,az):
  vel_msg = Twist()
  vel_msg.linear.x = lx
  vel_msg.linear.y = ly
  vel_msg.linear.z = lz
  vel_msg.angular.x = ax
  vel_msg.angular.y = ay
  vel_msg.angular.z = az
  pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
  pub.publish(vel_msg)


def Detect_And_Publish(frame):

  face_cascade = cv2.CascadeClassifier('../haarcascades/haarcascade_frontalface_default.xml')
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  faces = face_cascade.detectMultiScale(gray, 1.3, 5)

  for(x,y,w,h) in faces:
    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

  frame_height = frame.shape[0]
  frame_width = frame.shape[1]
  
  center_bias = 1.0
  distance_bias = 1.0
  
  if(len(faces)):
    (x,y,w,h) = faces[0]
    center_bias = float(x+w/2-frame_width/2) / float(frame_width)
    distance_bias = 0.3 - float(w) / float(frame_width)
    print 'distance_bias', distance_bias
    print 'center_bias', center_bias
  
  return center_bias, distance_bias

def Capture_And_Show():
  cap = cv2.VideoCapture(3)
  pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
  frame_count = 0
  count_for_stop = 0
  center_bias = 0
  distance_bias = 0
  while(1):
    ret, frame = cap.read()    # get a frame
    frame_count = (frame_count + 1)%40
    if frame_count == 0:
      re_center_bias, re_distance_bias = Detect_And_Publish(frame)
      
      if(re_center_bias!=1.0 and re_distance_bias!=1.0):
        #distance control
        if re_distance_bias > 0.1:
          distance_bias = 0.2
        elif re_distance_bias < 0:
          distance_bias = -0.2
        else:
          distance_bias = 0
        
        #center control
        if re_center_bias > 0.2:
        	center_bias = 0.15
        elif re_center_bias < -0.2:
        	center_bias = -0.15
        else:
        	center_bias = 0
        
        count_for_stop = 0
      else:
        count_for_stop = count_for_stop + 1

    if count_for_stop==2:
      distance_bias = 0
      center_bias = 0
   
    vel_msg = Twist(Vector3(distance_bias,0,0), Vector3(0,0,center_bias))
    pub.publish(vel_msg)
    cv2.imshow("capture", frame) # show a frame
    if cv2.waitKey(50) & 0xFF == ord('q'): # refresh rate
        break
  cap.release()
  cv2.destroyAllWindows()


def main():
  rospy.init_node('Subscribe_Image_Node')
  Capture_And_Show()
  rospy.spin()
  
  
if __name__ == '__main__':
  main()
