import sys
import rospy
import cv2
import numpy as np
from radar_msgs.msg import RngAzResp

def callback(msg):
    
    # Convert the RawPacketData msg to a numpy array
    arr = np.array(msg.data)
    arr = arr.reshape(
        (msg.layout.dim[0].size,
        msg.layout.dim[1].size,
        msg.layout.dim[2].size))
    rng_bins = msg.layout.dim[0].size
    az_bins = msg.layout.dim[1].size
    chirps = msg.layout.dim[2].size
    sent_time = msg.header.stamp

    #generate and display the image
    img = np.flip(arr[:,:,0])

    #convert to correct format
    img = (img * 255).astype(np.uint8)

    zoom = 5
    img = cv2.resize(img,
                             (img.shape[1] * zoom,
                             img.shape[0] * zoom))
    
    #display the image
    cv2.imshow("RngAzResp",img)
    cv2.waitKey(1)
    

def subscriber():
    
    sub = rospy.Subscriber('radar/RngAzResp_Array',RngAzResp,callback)

    rospy.init_node('DCA_RngAzResp_img_sub',anonymous=True)

    rospy.spin()
      
if __name__ == '__main__':
  try:
      subscriber()
  except rospy.ROSInterruptException:
      pass