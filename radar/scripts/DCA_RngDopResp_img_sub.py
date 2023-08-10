#!/usr/bin/python3
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    try:
       bridge = CvBridge()
       
       cv_img = bridge.imgmsg_to_cv2(data,"passthrough")

       cv2.imshow("RngDopResp",cv_img)

       cv2.waitKey(1)
    except CvBridgeError as e:
       print(e)
    

def subscriber():
    
    sub = rospy.Subscriber('radar/RngDopResp_Img',Image,callback)

    rospy.init_node('DCA_RngDopResp_img_sub',anonymous=True)

    rospy.spin()
      
if __name__ == '__main__':
  try:
      subscriber()
  except rospy.ROSInterruptException:
      pass