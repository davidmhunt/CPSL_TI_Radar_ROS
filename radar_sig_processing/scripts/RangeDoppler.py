#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from radar_msgs.msg import RngDopResp

#other modules
import numpy as np
from multiprocessing.connection import Client
from multiprocessing import connection
import json
import os

class RangeDopplerResponse():

    def __init__(self):

        config_path = rospy.get_param('~config_path')

        if os.path.isfile(config_path):
            self.config = self.parse_json(config_path)
        else:
            rospy.loginfo("could not find config at {}".format(config_path))
            return
        
        #initialize the ADC data cube publishers and connections
        self.range_dop_pubs = []
        self.range_dop_conns = []
        self.range_dop_pubs_and_conns_init()

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

    def range_dop_pubs_and_conns_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            #initialize the publisher
            pub_name = "radar_{}/RangeDopplerResponse/Array".format(i)
            self.range_dop_pubs.append(rospy.Publisher(pub_name,RngDopResp,queue_size=10))

            #initialize the subscriber
            rospy.loginfo("Waiting to connect to agent_{} RangeDoppler Listener at {}".format(i,radar_configs[i]["rang_dop_resp"]))
            address = ('localhost', radar_configs[i]["rang_dop_resp"])     # family is deduced to be 'AF_INET'
            authkey_str = "DCA1000_client"
            self.range_dop_conns.append(Client(address, authkey=authkey_str.encode()))
            rospy.loginfo("Connected to agent_{} RangeDoppler Listener at {}".format(i,radar_configs[i]["rang_dop_resp"]))

        return
            
    
    def run(self):
        
        while not rospy.is_shutdown():
            
            try:
                ready_cons = connection.wait(
                    self.range_dop_conns,timeout=1.0
                )

                if ready_cons:
                    for conn in ready_cons:
                        #get the index of the connection in the range_dop_conns list
                        idx = self.range_dop_conns.index(conn)

                        #receive the message
                        rng_dop_resp = conn.recv()

                        #generate an array message
                        msg = self._get_array_msg(rng_dop_resp)

                        #publish the message
                        self.range_dop_pubs[idx].publish(msg)
                else:
                    break
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
            except rospy.ROSInterruptException:
                break

    def _get_array_msg(self,rng_dop_resp:np.ndarray):
        """Publish the range doppler response to the array message

        Args:
            rng_dop_resp (np.ndarray): the range doppler response to be published
        """
        ##define the numpy header
        msg = RngDopResp()

        header = Header()
        header.stamp = rospy.Time.now()
        
        msg.header = header
        msg.data = rng_dop_resp.flatten().tolist()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "RngBin"
        msg.layout.dim[0].size = rng_dop_resp.shape[0]
        msg.layout.dim[0].stride = rng_dop_resp.shape[1]
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[1].label = "VelBin"
        msg.layout.dim[1].size = rng_dop_resp.shape[1]
        msg.layout.dim[1].stride = 1

        # Publish the message
        return msg

def main():
    rospy.init_node('RangeDopplerResponse',anonymous=True)
    
    try:
        RangeDopplerResponse()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()