import rospy
from radar_datasets.DCA1000 import DCA1000

def main():
    rospy.init_node('DCA1000_DatasetGenerator',anonymous=True)
    
    try:
        dataset_generator = DCA1000()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()