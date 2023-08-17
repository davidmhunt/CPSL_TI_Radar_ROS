#Way of transmitting complex data, but with a header so that the data can be time stamped

std_msgs/Header                 header
std_msgs/MultiArrayLayout       layout
int16[]                         real_data
int16[]                         complex_data