#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from radar_msgs.msg import ADCDataCube, RngAzResp
import numpy as np
import os

class DualTopicListener:
    def __init__(self):
        # Initialize subscribers 
        self.subscriber_radar = rospy.Subscriber("radar/ADCDataCube_Array", ADCDataCube, self.update_latest_radar, queue_size=10)
        
        # Use a timer to implement the desired frequency
        rospy.Timer(rospy.Duration(0.1), self.save_latest_radar_lidar)  # 2 Hz
        
        # Variables to store the latest data from the topics
        self.latest_data_radar:ADCDataCube = None

        #for saving the data
        self.dataset_folder = "/home/locobot/data"
        self.radar_data_folder = "radar"
        self.save_file_name = "frame"
        self.sample_idx = 10000 #start off on frame 1000 so that frames have the same numbering

    def update_latest_radar(self, data):
        self.latest_data_radar = data

    def save_latest_radar_lidar(self,event):
        
        self._save_latest_radar_data()

        #incrememt the sample index
        self.sample_idx += 1

    def _save_latest_radar_data(self):
        #get the latest radar message
        msg = self.latest_data_radar

        #get the real data
        real_data = np.array(msg.real_data)
        real_data = real_data.reshape(
            (msg.layout.dim[0].size,
            msg.layout.dim[1].size,
            msg.layout.dim[2].size)
        )

        #get the real data
        imag_data = np.array(msg.imag_data)
        imag_data = imag_data.reshape(
            (msg.layout.dim[0].size,
            msg.layout.dim[1].size,
            msg.layout.dim[2].size)
        )

        data = real_data + 1j * imag_data

        #save the radar data
        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_folder,self.radar_data_folder,file_name)
        np.save(path,data)

        # log receiving the array
        rx_channels = msg.layout.dim[0].size
        samples_per_chirp = msg.layout.dim[1].size
        chirps_per_frame = msg.layout.dim[2].size
        sent_time = msg.header.stamp

        out_status = "Received ADCDataCube: rx channels: {}, samples: {}, chirps: {}, time: {}".format(rx_channels,samples_per_chirp,chirps_per_frame,sent_time)
        rospy.loginfo(out_status)
        

if __name__ == '__main__':
    rospy.init_node('dataset_generator', anonymous=True)
    listener = DualTopicListener()
    rospy.spin()
