import rospy
from radar_msgs.msg import RawPacketData
import numpy as np

def callback(msg):
    # Convert the RawPacketData msg to a numpy array
    arr = np.array(msg.data)
    num_samps = msg.layout.dim[0].size
    sent_time = msg.header.stamp
    
    #arr = arr.reshape((msg.layout.dim[0].size, msg.layout.dim[1].size, msg.layout.dim[2].size))

    # log receiving the array
    out_status = "Received RawPacketData: samples: {}, time: {}".format(num_samps,sent_time)
    rospy.loginfo(out_status)
    return

def subscriber():
    rospy.init_node('DCA_RawPacketData_sub', anonymous=True)
    rospy.Subscriber('radar/RawPacketData_Array', RawPacketData, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
