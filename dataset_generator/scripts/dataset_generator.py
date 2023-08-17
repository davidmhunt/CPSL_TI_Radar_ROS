#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from radar_msgs.msg import RawPacketData, RngAzResp
import numpy as np
import os

class DualTopicListener:
    def __init__(self):
        # Initialize subscribers 
        self.subscriber_lidar = rospy.Subscriber("velodyne_points", PointCloud2, self.update_latest_lidar, queue_size=10)
        self.subscriber_radar = rospy.Subscriber("radar/RawPacketData_Array", RawPacketData, self.update_latest_radar, queue_size=10)
        
        # Use a timer to implement the desired frequency
        rospy.Timer(rospy.Duration(0.5), self.save_latest_radar_lidar)  # 2 Hz
        
        # Variables to store the latest data from the topics
        self.latest_data_lidar:PointCloud2 = None
        self.latest_data_radar:RawPacketData = None

        #for saving the data
        self.dataset_folder = "/home/locobot/data"
        self.radar_data_folder = "radar"
        self.lidar_data_folder = "lidar"
        self.save_file_name = "frame"
        self.sample_idx = 0

    def update_latest_lidar(self, data):
        self.latest_data_lidar = data

    def update_latest_radar(self, data):
        self.latest_data_radar = data

    def save_latest_radar_lidar(self,event):
        
        self._save_latest_radar_data()
        self._save_latest_lidar_data()

        #incrememt the sample index
        self.sample_idx += 1

    def _save_latest_radar_data(self):
        #get the latest radar message
        msg = self.latest_data_radar

        #convert to a numpy array
        arr = np.array(msg.data)
        num_samps = msg.layout.dim[0].size
        sent_time = msg.header.stamp

        #save the radar data
        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_folder,self.radar_data_folder,file_name)
        np.save(path,arr)

        # log receiving the array
        out_status = "Saved Radar RawPacketData: samples: {}, time: {}".format(num_samps,sent_time)
        rospy.loginfo(out_status)
    
    def _save_latest_lidar_data(self):
        
        #get the latest lidar message
        msg = self.latest_data_lidar
        sent_time = msg.header.stamp
        #convert to a numpy array
        gen = pc2.read_points(msg,skip_nans=True)
        point_cloud = np.array(list(gen))

        #save the pointcloud to a file
        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_folder,self.lidar_data_folder,file_name)
        np.save(path,point_cloud)

        # log receiving the array
        out_status = "Saved Lidar Point Cloud: points: {}, time: {}".format(np.shape(point_cloud)[0],sent_time)
        rospy.loginfo(out_status)
        pass

    def timer_callback(self, event):
        if self.latest_data_lidar:
            rospy.loginfo("From lidar: %s", self.latest_data_lidar.data)
            self.repub_lidar.publish(self.latest_data_lidar)
        if self.latest_data_radar:
            rospy.loginfo("From radar: %s", self.latest_data_radar.data)
            self.repub_radar.publish(self.latest_data_radar)
        

if __name__ == '__main__':
    rospy.init_node('dataset_generator', anonymous=True)
    listener = DualTopicListener()
    rospy.spin()
