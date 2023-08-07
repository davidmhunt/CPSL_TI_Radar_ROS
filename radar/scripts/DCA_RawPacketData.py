#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from radar_msgs.msg import RawPacketData

#other modules
import numpy as np
from multiprocessing.connection import Client


class RawPacketDataPub():

    def __init__(self):

        self.array_pub = rospy.Publisher('radar/RawPacketData_Array',RawPacketData,queue_size=20)

        self.rate = rospy.Rate(10)

        self.conn = None

        self._connect_to_radar()

    def run(self):

        while not rospy.is_shutdown():
            
            try:
                #receive message from DCA1000
                raw_packet_data = self.conn.recv() #gives np.array(dtype=np.int16)
                # Create a 3-dimensional numpy array
                #raw_packet_data = np.random.randint(low=0,high=2,size=128)

                #publish array_message
                self._publish_array_msg(raw_packet_data)

                #TODO: remove when finalizing implementation
                #self.rate.sleep()
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
    
    def _connect_to_radar(self):
        #TODO: add code to connect to Radar
        rospy.loginfo("Waiting to connect to Listener")
        address = ('localhost', 6001)     # family is deduced to be 'AF_INET'
        authkey_str = "DCA1000_client"
        self.conn = Client(address, authkey=authkey_str.encode())

        rospy.loginfo("Connected to Listener")

    def _publish_array_msg(self,raw_packet_data:np.ndarray):
        """Publish the range_az_resp to the array message

        Args:
            rng_az_resp (np.ndarray): the range_az_resp to be published
        """
        #define the numpy header
        msg = RawPacketData()

        header = Header()
        header.stamp = rospy.Time.now()
        
        msg.header = header
        msg.data = raw_packet_data.tolist()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "sample"
        msg.layout.dim[0].size = raw_packet_data.size
        msg.layout.dim[0].stride = 1

        # Publish the message
        self.array_pub.publish(msg)

def main():
    rospy.init_node('DCA_RawPacketData')
    RawPacketData_pub = RawPacketDataPub()
    try:
        RawPacketData_pub.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()