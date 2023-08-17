import rospy
from radar_msgs.msg import ADCDataCube
import numpy as np

def callback(msg):
    #get the real data
    real_data = np.array(msg.real_data)
    real_data = real_data.reshape(
        (msg.layout.dim[0].size,
         msg.layout.dim[1].size,
         msg.layout.dim[2].size)
    )

    #get the real data
    imag_data = np.array(msg.complex_data)
    imag_data = imag_data.reshape(
        (msg.layout.dim[0].size,
         msg.layout.dim[1].size,
         msg.layout.dim[2].size)
    )

    data = real_data + 1j * imag_data

    rx_channels = msg.layout.dim[0]
    samples_per_chirp = msg.layout.dim[1]
    chirps_per_frame = msg.layout.dim[2]

    sent_time = msg.header.stamp
    
    #arr = arr.reshape((msg.layout.dim[0].size, msg.layout.dim[1].size, msg.layout.dim[2].size))

    # log receiving the array
    out_status = "Received ADCDataCube: rx channels: {}, samples: {}, chirps: {}, time: {}".format(rx_channels,samples_per_chirp,chirps_per_frame,sent_time)
    rospy.loginfo(out_status)
    return

def subscriber():
    rospy.init_node('DCA_ADCDataCube_sub', anonymous=True)
    rospy.Subscriber('radar/ADCDataCube_Array', ADCDataCube, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
