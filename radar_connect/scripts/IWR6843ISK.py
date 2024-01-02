#!/usr/bin/env python3

#ROS modules
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

#other modules
import numpy as np
from multiprocessing.connection import Client
from multiprocessing import connection
import json
import os

class IWR6843ISK():

    def __init__(self):

        config_path = rospy.get_param('~config_path')

        if os.path.isfile(config_path):
            self.config = self.parse_json(config_path)
        else:
            rospy.loginfo("could not find config at {}".format(config_path))
            return
        
        #initialize the ADC data cube publishers and connections
        self.point_cloud_pubs = []
        self.point_cloud_conns = []
        self.point_cloud_pubs_and_conns_init()

        self.run()
        
    def parse_json(self,json_file_path):
        """Read a json file at the given path and return a json object

        Args:
            json_file_path (str): path to the JSON file

        Returns:
            _type_: json
        """

        # open the JSON file
        f = open(json_file_path)
        content = ""
        for line in f:
            content += line
        return json.loads(content)

    def point_cloud_pubs_and_conns_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            #initialize the publisher
            pub_name = "radar_{}/PointCloud/Detections".format(i)
            self.point_cloud_pubs.append(rospy.Publisher(pub_name,PointCloud2,queue_size=10))

            #initialize the subscriber
            rospy.loginfo("Waiting to connect to agent_{} PointCloud Listener at {}".format(i,radar_configs[i]["point_cloud"]))
            address = ('localhost', radar_configs[i]["point_cloud"])     # family is deduced to be 'AF_INET'
            authkey_str = "TLV_client"
            self.point_cloud_conns.append(Client(address, authkey=authkey_str.encode()))
            rospy.loginfo("Connected to agent_{} PointCloud Listener at {}".format(i,radar_configs[i]["point_cloud"]))

        return
            
    
    def run(self):
        
        while not rospy.is_shutdown():
            
            try:
                ready_cons = connection.wait(
                    self.point_cloud_conns,timeout=1.0
                )

                if ready_cons:
                    for conn in ready_cons:
                        #get the index of the connection in the adc_cube_conns list
                        idx = self.point_cloud_conns.index(conn)

                        #receive the message
                        adc_data_cube = conn.recv()

                        #generate an array message
                        msg = self._get_point_cloud_msg(adc_data_cube)

                        #publish the message
                        self.point_cloud_pubs[idx].publish(msg)
                else:
                    break
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
            except rospy.ROSInterruptException:
                break

    def _get_point_cloud_msg(self,point_cloud:np.ndarray):
        """Publish the adc_data_cube to the array message

        Args:
            adc_data_cube (np.ndarray): the adc_data_cube to be published
        """
        # Create header for point cloud message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'Radar'

        # Create fields for point cloud message
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('vel', 12, PointField.FLOAT32, 1)]

        # Convert numpy array to string of bytes
        data_bytes = point_cloud.tobytes()

        # Create point cloud message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = point_cloud.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = len(data_bytes)
        msg.data = data_bytes

        # Publish the message
        return msg

def main():
    rospy.init_node('IWR6843ISK',anonymous=True)
    
    try:
        IWR6843ISK()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()