#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension

#custom message for publishng the range azimuth response
from radar_msgs.msg import RngDopResp

#other modules
import numpy as np
from multiprocessing.connection import Client


class MultiPathPub:

    def __init__(self):
        
        #create publishers for the array and the image
        self.array_pub = rospy.Publisher('radar/MultiPath_Array', RngDopResp, queue_size=20)

        self.rate = rospy.Rate(10)

        self.conn = None
        self._connect_to_radar()

    def run(self):

        while not rospy.is_shutdown():
            
            try:
                #receive message from DCA1000
                multi_path = self.conn.recv() #gives np.array(dtype=np.float32)

                #publish array_message
                self._publish_array_msg(multi_path)
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
    
    def _connect_to_radar(self):
        #TODO: add code to connect to Radar
        rospy.loginfo("Waiting to connect to Listener")
        address = ('localhost', 6006)     # family is deduced to be 'AF_INET'
        authkey_str = "DCA1000_client"
        self.conn = Client(address, authkey=authkey_str.encode())

        rospy.loginfo("Connected to Listener")
        pass

        #TODO: uncomment when connecting to the radar

    def _publish_array_msg(self,multi_path:np.ndarray):
        """Publish the multi_path to the array message

        Args:
            multi_path (np.ndarray): the multi_path to be published
        """
        #define the numpy header
        msg = RngDopResp()

        header = Header()
        header.stamp = rospy.Time.now()
        
        msg.header = header
        msg.data = multi_path.flatten().tolist()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "RngBin"
        msg.layout.dim[0].size = multi_path.shape[0]
        msg.layout.dim[0].stride = multi_path.shape[1]
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[1].label = "AzBin"
        msg.layout.dim[1].size = multi_path.shape[1]
        msg.layout.dim[1].stride = 1

        # Publish the message
        self.array_pub.publish(msg)



def main():
    rospy.init_node('DCA_MultiPath',anonymous=True)
    MultiPath_pub = MultiPathPub()
    try:
        MultiPath_pub.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

