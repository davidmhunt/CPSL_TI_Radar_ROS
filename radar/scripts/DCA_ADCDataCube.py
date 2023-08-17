#!/usr/bin/env python3

#ROS modules
import rospy
from std_msgs.msg import Header,MultiArrayDimension
from radar_msgs.msg import ADCDataCube

#other modules
import numpy as np
from multiprocessing.connection import Client


class ADCDataCubePub():

    def __init__(self):

        self.array_pub = rospy.Publisher('radar/ADCDataCube_Array',ADCDataCube,queue_size=20)

        self.rate = rospy.Rate(10)

        self.conn = None

        self._connect_to_radar()

    def run(self):

        while not rospy.is_shutdown():
            
            try:
                #receive message from DCA1000
                adc_data_cube = self.conn.recv() #gives np.array(dtype=np.int16)
                # Create a 3-dimensional numpy array
                #adc_data_cube = np.random.randint(low=0,high=2,size=128)

                #publish array_message
                self._publish_array_msg(adc_data_cube)

                #TODO: remove when finalizing implementation
                #self.rate.sleep()
            
            except EOFError:
                rospy.loginfo("Listener connection was closed by Radar")
                break
    
    def _connect_to_radar(self):
        #TODO: add code to connect to Radar
        rospy.loginfo("Waiting to connect to Listener")
        address = ('localhost', 6001)     # family is deduced to be 'AF_INET'
        authkey_str = "DCA1000_client"
        self.conn = Client(address, authkey=authkey_str.encode())

        rospy.loginfo("Connected to Listener")

    def _publish_array_msg(self,adc_data_cube:np.ndarray):
        """Publish the range_az_resp to the array message

        Args:
            adc_data_cube (np.ndarray): the adc_data_cube to be published
        """
        #define the numpy header
        msg = ADCDataCube()

        header = Header()
        header.stamp = rospy.Time.now()
        
        msg.header = header

        #package real part of the data
        msg.real_data = adc_data_cube.real.flatten().tolist()

        #package complex part of the data
        msg.complex_data = adc_data_cube.imag.flatten().tolist()

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
        self.array_pub.publish(msg)

def main():
    rospy.init_node('DCA_ADCDataCube')
    ADCDataCube_pub = ADCDataCubePub()
    try:
        ADCDataCube_pub.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()