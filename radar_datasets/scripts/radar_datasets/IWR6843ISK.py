#!/usr/bin/env python3

#ROS modules
import rospy
from sensor_msgs.msg import PointCloud2

#other modules
import numpy as np
import os

from radar_datasets._DatasetGenerator import _DatasetGenerator

class IWR6843ISK(_DatasetGenerator):

    def __init__(self):
        super().__init__()

#subscribers to the ADC cube message
    def radar_data_subs_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            self.radar_data_subs.append(
                rospy.Subscriber(
                    "radar_{}/PointCloud/Detections".format(i),
                    PointCloud2,
                    self.radar_data_sub_callback,
                    callback_args=(i))
            )

            self.radar_data_latest.append(None) #initialize an empty message

    def radar_data_save_to_file(self):

        file_name = "{}_{}.npy".format(self.save_file_name,self.sample_idx)

        for i in range(len(self.radar_data_folders)):

            folder = self.radar_data_folders[i]
            msg = self.radar_data_latest[i]

            data_bytes = msg.data
            num_fields = msg.point_step // 4

            data = np.frombuffer(data_bytes,dtype=np.float32)

            data = data.reshape((-1,num_fields))

            path = os.path.join(self.dataset_path,folder,file_name)
            np.save(path,data)

            out_status = "Received {} detections from IWR6843ISK".format(data.shape[0])
            rospy.loginfo(out_status)