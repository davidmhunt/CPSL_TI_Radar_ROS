#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from radar_msgs.msg import ADCDataCube

#other modules
import numpy as np
from multiprocessing.connection import Client
from multiprocessing import connection
import json
import os

class DCA():

    def __init__(self):

        config_path = rospy.get_param('~config_path')

        if os.path.isfile(config_path):
            self.config = self.parse_json(config_path)
        else:
            rospy.loginfo("could not find config at {}".format(config_path))
            return
        
        #initialize the ADC data cube publishers and connections
        self.adc_cube_pubs = []
        self.adc_cube_conns = []
        self.adc_cube_pubs_and_conns_init()

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

    def adc_cube_pubs_and_conns_init(self):

        radar_configs = self.config["radars"]

        for i in range(len(radar_configs)):
            #initialize the publisher
            pub_name = "radar_{}/ADCDataCube/Array".format(i)
            self.adc_cube_pubs.append(rospy.Publisher(pub_name,ADCDataCube,queue_size=10))

            #initialize the subscriber
            rospy.loginfo("Waiting to connect to agent_{} ADCDataCube Listener at {}".format(i,radar_configs[i]["ADC_cube"]))
            address = ('localhost', radar_configs[i]["ADC_cube"])     # family is deduced to be 'AF_INET'
            authkey_str = "DCA1000_client"
            self.adc_cube_conns.append(Client(address, authkey=authkey_str.encode()))
            rospy.loginfo("Connected to agent_{} ADCDataCube Listener at {}".format(i,radar_configs[i]["ADC_cube"]))

        return
            
    
    def run(self):
        
        while not rospy.is_shutdown():
            
            try:
                ready_cons = connection.wait(
                    self.adc_cube_conns,timeout=1.0
                )

                if ready_cons:
                    for conn in ready_cons:
                        #get the index of the connection in the adc_cube_conns list
                        idx = self.adc_cube_conns.index(conn)

                        #receive the message
                        adc_data_cube = conn.recv()

                        #generate an array message
                        msg = self._get_array_msg(adc_data_cube)

                        #publish the message
                        self.adc_cube_pubs[idx].publish(msg)
                else:
                    break
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
            except rospy.ROSInterruptException:
                break

    def _get_array_msg(self,adc_data_cube:np.ndarray):
        """Publish the adc_data_cube to the array message

        Args:
            adc_data_cube (np.ndarray): the adc_data_cube to be published
        """
        #define the numpy header
        msg = ADCDataCube()

        header = Header()
        header.stamp = rospy.Time.now()
        
        msg.header = header

        #package real part of the data
        msg.real_data = adc_data_cube.real.astype(np.int16).flatten().tolist()

        #package complex part of the data
        msg.imag_data = adc_data_cube.imag.astype(np.int16).flatten().tolist()

        #define the layout
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "rx_channel"
        msg.layout.dim[0].size = adc_data_cube.shape[0]
        msg.layout.dim[0].stride = adc_data_cube.shape[1] * adc_data_cube.shape[2]
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[1].label = "sample"
        msg.layout.dim[1].size = adc_data_cube.shape[1]
        msg.layout.dim[1].stride = adc_data_cube.shape[2]
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[2].label = "chirp"
        msg.layout.dim[2].size = adc_data_cube.shape[2]
        msg.layout.dim[2].stride = 1

        # Publish the message
        return msg

def main():
    rospy.init_node('DCA1000',anonymous=True)
    
    try:
        DCA()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()