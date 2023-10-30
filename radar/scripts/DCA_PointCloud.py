#!/usr/bin/env python3

#ROS modules
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

#other modules
import numpy as np
from multiprocessing.connection import Client


class PointCloudPub:

    def __init__(self):
        
        #create publishers for the array and the image
        self.point_cloud_pub = rospy.Publisher('radar/PointCloud', PointCloud2, queue_size=20)

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
                self._publish_point_cloud_msg(rng_az_resp)
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
    
    def _connect_to_radar(self):
        rospy.loginfo("Waiting to connect to Listener")
        address = ('localhost', 6005)     # family is deduced to be 'AF_INET'
        authkey_str = "DCA1000_client"
        self.conn = Client(address, authkey=authkey_str.encode())

        rospy.loginfo("Connected to Listener")
        pass


    def _publish_point_cloud_msg(self,point_cloud:np.ndarray):
        """Publish the range_az_resp to the array message

        Args:
            point_cloud (np.ndarray): the point_cloud to be published
        """
        
        
        # Create header for point cloud message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'Radar'

        # Create fields for point cloud message
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]

        # Convert numpy array to string of bytes
        data_str = point_cloud.tobytes()

        # Create point cloud message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = point_cloud.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = len(data_str)
        msg.data = data_str

        # Publish the message
        self.point_cloud_pub.publish(msg)



def main():
    rospy.init_node('DCA_PointCloud',anonymous=True)
    PointCloud_pub = PointCloudPub()
    try:
        PointCloud_pub.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

