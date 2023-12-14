import sys
import rospy
import cv2
import numpy as np
from radar_msgs.msg import RngDopResp
import os
import json

class RangeDopplerResponseView():

    def __init__(self):
        
        #parameter for the configuration path
        config_path = rospy.get_param('~config_path')
        if os.path.isfile(config_path):
            self.config = self.parse_json(config_path)
        else:
            rospy.loginfo("could not find config at {}".format(config_path))
            return
        
        #parameter allowing option to only view a specific radar
        self.view_only = int(rospy.get_param('~view_only',default=None))

        #initialize the number subscribers
        self.range_dop_subs = []
        self.range_dop_subs_init()

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
    
    def range_dop_subs_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            if self.view_only != -1 and (i != self.view_only):
                self.range_dop_subs.append(None)
            else:
                self.range_dop_subs.append(
                    rospy.Subscriber(
                        "radar_{}/RangeDopplerResponse/Array".format(i),
                        RngDopResp,
                        self.range_dop_sub_callback,
                        callback_args=(i))
                )



    def range_dop_sub_callback(self,msg,radar_idx):
        # Convert the RawPacketData msg to a numpy array
        arr = np.array(msg.data)
        arr = arr.reshape(
            (msg.layout.dim[0].size,
            msg.layout.dim[1].size))
        rng_bins = msg.layout.dim[0].size
        dop_bins = msg.layout.dim[1].size
        sent_time = msg.header.stamp

        #generate the image
        #correct the orientation
        img = np.flip(arr)

        #convert to correct format
        img = (img * 255).astype(np.uint8)

        #output a larger image
        zoom = 5
        img = cv2.resize(
            img,
            (img.shape[1] * zoom,
            img.shape[0] * zoom)
        )
        
        #display the image
        cv2.imshow("Radar {}: Range Doppler Response".format(radar_idx),img)
        cv2.waitKey(1)
  
      
if __name__ == '__main__':
  try:
    #initialize the ROS node
    rospy.init_node('RangeDopplerResponseView',anonymous=True)

    rng_dop_view = RangeDopplerResponseView()

    rospy.spin()
  except rospy.ROSInterruptException:
    pass