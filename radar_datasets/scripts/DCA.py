#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from radar_msgs.msg import ADCDataCube

#other modules
import numpy as np
import json
import os

class DCA():

    def __init__(self):

        #load the configuration
        config_path = rospy.get_param('~config_path')
        if os.path.isfile(config_path):
            self.config = self.parse_json(config_path)
        else:
            rospy.loginfo("could not find config at {}".format(config_path))
            return
        #determine if recording a dataset for radar+lidar or radar only
        self.radar_only = rospy.get_param('~radar_only',default=False)
        
        #determine the dataset path
        self.dataset_path = rospy.get_param('~dataset_path',default="/home/locobot/data")
        self.radar_data_folders = []
        self.lidar_data_folder = "lidar"
        self.save_file_name = "frame"
        self.sample_idx = 10000 #start at frame 10000
        self.init_radar_lidar_data_folders()

        #implement a timer for sampling data at the latest frequency (defaults to 50ms)
        frame_rate = float(rospy.get_param('~frame_rate',default=20))
        rospy.Timer(
            rospy.Duration(1/frame_rate),
            self.save_latest_radar_lidar
        )


        #initialize the subscriber for the adc data cubes
        self.radar_data_subs = []
        self.radar_data_latest = []
        self.radar_data_subs_init()

        #initialize the subscriber for the lidar data
        if not self.radar_only:
            self.lidar_data_latest:PointCloud2 = None
            self.lidar_data_sub = rospy.Subscriber(
                "velodyne_points", 
                PointCloud2, 
                self.lidar_data_callback,
                queue_size=10)
        
        

#parsing a json configuration file 
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

#initializing the folders used to generate a dataset
    def check_for_directory(self,path, clear_contents = False):
        """Checks to see if a directory exists, 
        if the directory does not exist, attepts to create the directory.
        If it does exist, optionally removes all files

        Args:
            path (str): path to the directory to create
            clear_contents (bool, optional): removes all contents in the directory on True. Defaults to False.
        """

        if os.path.isdir(path):
            rospy.loginfo("DatasetGenerator.check_for_directory: found directory {}".format(path))

            if clear_contents:
                rospy.loginfo("DatasetGenerator.check_for_directory: clearing contents of {}".format(path))

                #clear the contents
                for file in os.listdir(path):
                    file_path = os.path.join(path,file)

                    try:
                        if os.path.isfile(file_path):
                            os.remove(file_path)
                    except Exception as e:
                        print("Failed to delete {}".format(path))
        else:
            rospy.loginfo("DatasetGenerator.check_for_directory: creating directory {}".format(path))
            os.makedirs(path)
        return
    
    def init_radar_lidar_data_folders(self):

        #initialize the lidar data folder
        if not self.radar_only:
            path = os.path.join(self.dataset_path,self.lidar_data_folder)
            self.check_for_directory(path,clear_contents=True)

        #initialize the radar data folders
        for i in range(len(self.config["radars"])):
            folder_name = "radar_{}".format(i)
            
            #create/clear the dataset folder
            path = os.path.join(self.dataset_path,folder_name)
            self.check_for_directory(path,clear_contents=True)

            #append the folder name
            self.radar_data_folders.append(folder_name)

#subscribers to the ADC cube message
    def radar_data_subs_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            self.radar_data_subs.append(
                rospy.Subscriber(
                    "radar_{}/ADCDataCube/Array".format(i),
                    ADCDataCube,
                    self.radar_data_sub_callback,
                    callback_args=(i))
            )

            self.radar_data_latest.append(None) #initialize an empty message
    

    def radar_data_sub_callback(self,msg,radar_idx):
        
        self.radar_data_latest[radar_idx] = msg
        return
    
#subscribers to the lidar message
    def lidar_data_callback(self,msg):

        self.lidar_data_latest = msg
        return

#saving the data to a file
    def save_latest_radar_lidar(self,event):
        
        if self.check_latest_data():

            #save the lidar data
            if not self.radar_only:
                self.lidar_data_save_to_file()

            #save the radar data
            self.radar_data_save_to_file()

            self.sample_idx += 1

            return
        else:
            return

    def check_latest_data(self):

        #check lidar data valid
        if (not self.radar_only) and self.lidar_data_latest is None:
            return False
        
        for i in range(len(self.radar_data_latest)):
            if self.radar_data_latest[i] is None:
                return False
        
            else:
                return True
        
    def lidar_data_save_to_file(self):

        #get the latest lidar data message
        msg = self.lidar_data_latest
        sent_time = msg.header.stamp
        #convert to a numpy array
        gen = pc2.read_points(msg,skip_nans=True)
        point_cloud = np.array(list(gen))

        #save the pointcloud to a file
        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_path,self.lidar_data_folder,file_name)
        np.save(path,point_cloud)

        # log receiving the array
        out_status = "Saved Lidar Point Cloud: points: {}, time: {}".format(np.shape(point_cloud)[0],sent_time)
        rospy.loginfo(out_status)

    def radar_data_save_to_file(self):

        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)

        for i in range(len(self.radar_data_folders)):

            folder = self.radar_data_folders[i]
            msg = self.radar_data_latest[i]

            #get the real data
            real_data = np.array(msg.real_data)
            real_data = real_data.reshape(
                (msg.layout.dim[0].size,
                msg.layout.dim[1].size,
                msg.layout.dim[2].size)
            )

            #get the imag data
            imag_data = np.array(msg.imag_data)
            imag_data = imag_data.reshape(
                (msg.layout.dim[0].size,
                msg.layout.dim[1].size,
                msg.layout.dim[2].size)
            )

            data = real_data + 1j * imag_data

            path = os.path.join(self.dataset_path,folder,file_name)
            np.save(path,data)

            # log receiving the array
            rx_channels = msg.layout.dim[0].size
            samples_per_chirp = msg.layout.dim[1].size
            chirps_per_frame = msg.layout.dim[2].size
            sent_time = msg.header.stamp

            out_status = "Received ADCDataCube from radar {}: rx channels: {}, samples: {}, chirps: {}, time: {}".format(i,rx_channels,samples_per_chirp,chirps_per_frame,sent_time)
            rospy.loginfo(out_status)