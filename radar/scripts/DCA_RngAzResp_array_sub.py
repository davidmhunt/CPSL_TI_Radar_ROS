import rospy
from radar_msgs.msg import RngAzResp
import numpy as np

def callback(msg):
    # Convert the RawPacketData msg to a numpy array
    arr = np.array(msg.data)
    arr = arr.reshape(
        (msg.layout.dim[0].size,
        msg.layout.dim[1].size,
        msg.layout.dim[2].size))
    rng_bins = msg.layout.dim[0].size
    az_bins = msg.layout.dim[1].size
    chirps = msg.layout.dim[2].size
    sent_time = msg.header.stamp

    # log receiving the array
    out_status = "Received RawPacketData: dimmensions: ({},{},{}), time: {}".format(
        rng_bins,az_bins,chirps,sent_time)
    rospy.loginfo(out_status)
    return

def subscriber():
    rospy.init_node('DCA_RngAzResp_array_sub', anonymous=True)
    rospy.Subscriber('radar/RngAzResp_Array', RngAzResp, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
