#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from sensor_msgs.msg import PointCloud2, PointField
from radar_msgs.msg import RngAzResp

#other modules
import numpy as np
from multiprocessing.connection import Listener,Client
from multiprocessing import connection
import json
import os

class RadNav():

    def __init__(self):

        config_path = rospy.get_param('~config_path')

        if os.path.isfile(config_path):
            self.config = self.parse_json(config_path)
        else:
            rospy.loginfo("could not find config at {}".format(config_path))
            return
        
        #initialize variable to see how many radnav models to use
        self.compute_only_on = rospy.get_param('~compute_only_on',default=0)
        
        #initialize the radnav model
        # self._rad_nav_model_listeners = []
        self._rad_nav_model_conns = []
        self._rad_nav_model_init()
        
        #initialize the subscriber for radnav model
        self.range_az_subs = []
        self.range_az_subs_init()

        #initialize the ADC data cube publishers and connections
        self.point_cloud_pubs = []
        self.range_az_conns = []
        self.point_cloud_pubs_and_conns_init()
        
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

#handle communication to/from the radcloud model
    def _rad_nav_model_init(self):

        radar_configs = self.config["radars"]

        # get the authentication string
        authkey_str = "DCA1000_client"
        authkey = authkey_str.encode()

        #wait for a new connection
        for i in range(len(radar_configs)):
            if self.compute_only_on != -1 and (i != self.compute_only_on):
                self._rad_nav_model_conns.append(None)
            else:
                rad_nav_model_addr = (
                    "localhost",
                    int(radar_configs[i]["rad_nav_model"]))
                
                self._rad_nav_model_conns.append(
                    Client(rad_nav_model_addr, authkey=authkey))

                # #for listeners (servers that Clients connect to)
                # self._rad_nav_model_listeners.append(
                #     Listener(rad_nav_model_addr, authkey=authkey))
                # self._rad_nav_model_conns.append(
                #     self._rad_nav_model_listeners[i].accept())

    def rad_nav_model_process_range_az_resp(self,radar_idx,range_azimuth_response:np.ndarray):

        #send range doppler response to RadNav model
        try:
            self._rad_nav_model_conns[radar_idx].send(range_azimuth_response)
        except ConnectionResetError:
            rospy.loginfo("RadNav model:Error sending packet to model")
            rospy.signal_shutdown("RadNave model node experienced error sending packet to model")
        #receive generated point cloud from rad nav model
        try:
            point_cloud = np.float32(self._rad_nav_model_conns[radar_idx].recv())
        except EOFError:
            rospy.loginfo("RadNav model: Error receiving packets from model")
            rospy.signal_shutdown("RadNave model node experienced error receiving packet from model")
        return point_cloud
    
#subscribers to the range azimuth message
    def range_az_subs_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            if self.compute_only_on != -1 and (i != self.compute_only_on):
                self.range_az_subs.append(None)
            else:
                self.range_az_subs.append(
                    rospy.Subscriber(
                        "radar_{}/RangeAzimuthResponse/Array".format(i),
                        RngAzResp,
                        self.range_az_sub_callback,
                        callback_args=(i))
                )

    def range_az_sub_callback(self,msg,radar_idx):
        
        # Convert the RawPacketData msg to a numpy array
        range_az_resp = np.array(msg.data)
        range_az_resp = range_az_resp.reshape(
            (msg.layout.dim[0].size,
            msg.layout.dim[1].size,
            msg.layout.dim[2].size))
        rng_bins = msg.layout.dim[0].size
        az_bins = msg.layout.dim[1].size
        chirps = msg.layout.dim[2].size
        sent_time = msg.header.stamp

        #process the rad-nav model
        point_cloud = self.rad_nav_model_process_range_az_resp(radar_idx,range_az_resp)

        #publish the point cloud
        msg = self._get_point_cloud_msg(point_cloud)

        self.point_cloud_pubs[radar_idx].publish(msg)

#publishing the processed point cloud message
    def point_cloud_pubs_and_conns_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            #initialize the publisher
            pub_name = "radar_{}/PointCloud".format(i)
            self.point_cloud_pubs.append(rospy.Publisher(pub_name,PointCloud2,queue_size=20))
        return
    
    def _get_point_cloud_msg(self,point_cloud:np.ndarray):
        """Publish the range_az_resp to the array message

        Args:
            point_cloud (np.ndarray): the point_cloud to be published
        """
        
        
        # Create header for point cloud message
        header = Header()
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

        return msg

def main():
    rospy.init_node('RadNav',anonymous=True)
    
    try:
        rad_nav_node = RadNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()