#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

#custom message for publishng the range azimuth response
from radar_msgs.msg import RngAzResp

#other modules
import numpy as np
import cv2
from multiprocessing.connection import Client


class RngAzRespPub:

    def __init__(self):
        
        #create publishers for the array and the image
        self.array_pub = rospy.Publisher('radar/RngAzResp_Array', RngAzResp, queue_size=20)
        self.img_pub = rospy.Publisher('radar/RngAzResp_Img',Image,queue_size=20)
        
        #define bridge for converting between ROS image messages and cv2 data
        self.bridge = CvBridge()

        self.rate = rospy.Rate(10)

        self.conn = None
        self._connect_to_radar()

    def run(self):

        while not rospy.is_shutdown():
            
            try:
                #receive message from DCA1000
                rng_az_resp = self.conn.recv() #gives np.array(dtype=np.float32)
                # Create a 3-dimensional numpy array
                #rng_az_resp = np.random.rand(128,256,40)

                #publish array_message
                self._publish_array_msg(rng_az_resp)

                #publish image message\
                self._publish_img_msg(rng_az_resp)

                #TODO: remove when finalizing implementation
                #self.rate.sleep()
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
    
    def _connect_to_radar(self):
        #TODO: add code to connect to Radar
        rospy.loginfo("Waiting to connect to Listener")
        address = ('localhost', 6002)     # family is deduced to be 'AF_INET'
        authkey_str = "DCA1000_client"
        self.conn = Client(address, authkey=authkey_str.encode())

        rospy.loginfo("Connected to Listener")
        pass

        #TODO: uncomment when connecting to the radar

    def _publish_array_msg(self,rng_az_resp:np.ndarray):
        """Publish the range_az_resp to the array message

        Args:
            rng_az_resp (np.ndarray): the range_az_resp to be published
        """
        #define the numpy header
        msg = RngAzResp()

        header = Header()
        header.stamp = rospy.Time.now()
        
        msg.header = header
        msg.data = rng_az_resp.flatten().tolist()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "RngBin"
        msg.layout.dim[0].size = rng_az_resp.shape[0]
        msg.layout.dim[0].stride = rng_az_resp.shape[1] * rng_az_resp.shape[2]
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[1].label = "AzBin"
        msg.layout.dim[1].size = rng_az_resp.shape[1]
        msg.layout.dim[1].stride = rng_az_resp.shape[2]
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[2].label = "Chirp"
        msg.layout.dim[2].size = rng_az_resp.shape[2]
        msg.layout.dim[2].stride = 1

        # Publish the message
        self.array_pub.publish(msg)
    
    def _publish_img_msg(self,rng_az_resp:np.ndarray):
        """Publish an image of the first range_az_resp to the img message

        Args:
            rng_az_resp (np.ndarray): the range_az_resp to be published
        """

        try:
            #correct the orientation
            img = np.flip(rng_az_resp[:,:,0])

            #convert to correct format
            img = (img * 255).astype(np.uint8)

            #output a larger image
            zoom = 5
            img = cv2.resize(img,
                             (img.shape[1] * zoom,
                             img.shape[0] * zoom))

            #convert to img_msg
            img_msg = self.bridge.cv2_to_imgmsg(img,"passthrough")

            self.img_pub.publish(img_msg)
        except CvBridgeError as e:
            print(e)



def main():
    rospy.init_node('DCA_RngAzResp',anonymous=True)
    RngAzResp_pub = RngAzRespPub()
    try:
        RngAzResp_pub.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

