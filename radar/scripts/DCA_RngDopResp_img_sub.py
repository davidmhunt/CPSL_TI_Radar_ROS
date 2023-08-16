#!/usr/bin/python3
import sys
import rospy
import cv2
import numpy as np
from radar_msgs.msg import RngDopResp

def callback(msg):
    
    # Convert the RawPacketData msg to a numpy array
    arr = np.array(msg.data)
    arr = arr.reshape(
        (msg.layout.dim[0].size,
        msg.layout.dim[1].size))
    rng_bins = msg.layout.dim[0].size
    dop_bins = msg.layout.dim[1].size
    sent_time = msg.header.stamp

    #generate the image
    #correct the orientation
    img = np.flip(arr)

    #convert to correct format
    img = (img * 255).astype(np.uint8)

    #output a larger image
    zoom = 5
    img = cv2.resize(img,
                        (img.shape[1] * zoom,
                        img.shape[0] * zoom))
    
    #show the image
    cv2.imshow("RngDopResp",img)

    cv2.waitKey(1)
    

def subscriber():
    
    sub = rospy.Subscriber('radar/RngDopResp_Array',RngDopResp,callback)

    rospy.init_node('DCA_RngDopResp_img_sub',anonymous=True)

    rospy.spin()
      
if __name__ == '__main__':
  try:
      subscriber()
  except rospy.ROSInterruptException:
      pass