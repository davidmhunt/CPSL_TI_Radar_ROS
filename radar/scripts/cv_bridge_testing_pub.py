#!/usr/bin/python3
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def cv_bridge_testing():
    
    pub = rospy.Publisher('cv_bridge_img',Image,queue_size=10)

    rospy.init_node('cv_bridge_testing_pub',anonymous=True)

    rate = rospy.Rate(10)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        
        try:
          #generate random image
          img = np.random.rand(128,256)

          #convert to uint 8
          img = (img * 255).astype(np.uint8)

          img_msg = bridge.cv2_to_imgmsg(img,"passthrough")

          pub.publish(img_msg)

          rate.sleep()
        
        except CvBridgeError as e:
           print(e)
           break
      
if __name__ == '__main__':
  try:
      cv_bridge_testing()
  except rospy.ROSInterruptException:
      pass