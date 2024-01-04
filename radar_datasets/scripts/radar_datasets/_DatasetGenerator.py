#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from sensor_msgs.msg import PointCloud2,Image
import sensor_msgs.point_cloud2 as pc2
from radar_msgs.msg import ADCDataCube
from cv_bridge import CvBridge
import cv2

#other modules
import numpy as np
import json
import os

class _DatasetGenerator():

    def __init__(self):

        #load the configuration
        config_path = rospy.get_param('~config_path')
        if os.path.isfile(config_path):
            self.config = self.parse_json(config_path)
        else:
            rospy.loginfo("could not find config at {}".format(config_path))
            return
        #determine if recording a dataset for radar+lidar or radar only
        self.radar_enable = rospy.get_param('~radar_enable',default=True)
        self.lidar_enable = rospy.get_param('~lidar_enable',default=True)
        self.camera_enable = rospy.get_param('~camera_enable',default=False)

        #determine the dataset path
        self.dataset_path = rospy.get_param('~dataset_path',default="/home/locobot/data")
        self.radar_data_folders = []
        self.lidar_data_folder = "lidar"
        self.camera_data_folder = "camera"
        self.save_file_name = "frame"
        self.sample_idx = 10000 #start at frame 10000
        self.init_data_folders()

        #implement a timer for sampling data at the latest frequency (defaults to 50ms)
        frame_rate = float(rospy.get_param('~frame_rate',default=20))
        rospy.Timer(
            rospy.Duration(1/frame_rate),
            self.save_latest_data
        )

        #initialize the subscriber for the radar data
        if self.radar_enable:
            self.radar_data_subs = []
            self.radar_data_latest = []
            self.radar_data_subs_init()

        #initialize the subscriber for the lidar data
        if self.lidar_enable:
            self.lidar_data_latest:PointCloud2 = None
            self.lidar_data_sub = rospy.Subscriber(
                "velodyne_points", 
                PointCloud2, 
                self.lidar_data_callback,
                queue_size=10)
        
        if self.camera_enable:
            self.camera_data_latest = None
            self.camera_data_sub = rospy.Subscriber(
                "/usb_cam/image_raw",
                Image,
                self.camera_data_callback,
                queue_size=10
            )
        
        

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
    
    def init_data_folders(self):

        #initialize the lidar data folder
        if self.lidar_enable:
            path = os.path.join(self.dataset_path,self.lidar_data_folder)
            self.check_for_directory(path,clear_contents=True)

        if self.camera_enable:
            path = os.path.join(self.dataset_path,self.camera_data_folder)
            self.check_for_directory(path,clear_contents=True)

        #initialize the radar data folders
        if self.radar_enable:
            for i in range(len(self.config["radars"])):
                folder_name = "radar_{}".format(i)
                
                #create/clear the dataset folder
                path = os.path.join(self.dataset_path,folder_name)
                self.check_for_directory(path,clear_contents=True)

                #append the folder name
                self.radar_data_folders.append(folder_name)

#subscribers to the ADC cube message
    def radar_data_subs_init(self):
        """Function to initialize the radar data subscribers
        Initialized by the child classes
        """
        pass

    def radar_data_sub_callback(self,msg,radar_idx):
        
        self.radar_data_latest[radar_idx] = msg
        return
    
#subscribers to the lidar message
    def lidar_data_callback(self,msg):

        self.lidar_data_latest = msg
        return
    
    def camera_data_callback(self,msg:Image):
        
        self.camera_data_latest = msg

#saving the data to a file
    def save_latest_data(self,event):
        
        if self.check_latest_data():

            #save the lidar data
            if self.lidar_enable:
                self.lidar_data_save_to_file()

            #save the camera data
            if self.camera_enable:
                self.camera_data_save_to_file()

            #save the radar data
            if self.radar_enable:
                self.radar_data_save_to_file()

            self.sample_idx += 1

            return
        else:
            return

    def check_latest_data(self):

        #check lidar data valid
        if self.lidar_enable and self.lidar_data_latest is None:
            return False
        
        if self.camera_enable and self.camera_data_latest is None:
            return False

        if self.radar_enable:
            for i in range(len(self.radar_data_latest)):
                if self.radar_data_latest[i] is None:
                    return False
        
        #only if everything checks do we return True
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

    def camera_data_save_to_file(self):

        #get the latest camera data message
        msg = self.camera_data_latest

        #convert to numpy array
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")

        file_name = "{}_{}.png".format(self.save_file_name,self.sample_idx)
        path = os.path.join(self.dataset_path,self.camera_data_folder,file_name)
        cv2.imwrite(path,cv_image)

        out_status = "Saved Camera Frame"
        rospy.loginfo(out_status)


    def radar_data_save_to_file(self):
        """Function to save the latest radar data message to a file
        Defined by child class
        """
        pass