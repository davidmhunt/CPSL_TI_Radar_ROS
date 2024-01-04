import rospy
from radar_datasets.IWR6843ISK import IWR6843ISK

def main():
    rospy.init_node('IWR6843DatasetGenerator',anonymous=True)
    
    try:
        dataset_generator = IWR6843ISK()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()