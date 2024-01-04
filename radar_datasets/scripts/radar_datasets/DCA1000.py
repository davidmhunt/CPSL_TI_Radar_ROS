#!/usr/bin/env python3

#ROS modules
import rospy
from radar_msgs.msg import ADCDataCube

#other modules
import numpy as np
import os

from radar_datasets._DatasetGenerator import _DatasetGenerator

class DCA1000(_DatasetGenerator):

    def __init__(self):
        super().__init__()

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