#!/usr/bin/env python3

import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

# Global variables
bag = None


def scan_callback(msg):
    global bag
    try:
        bag.write("/scan", msg, msg.header.stamp)
    except Exception as e:
        rospy.logerr(f"Error converting and recording image: {str(e)}")
def main():
    global bag
    rospy.init_node("scan_bag_recorder", anonymous=True)
    
    # Path to save the bag file
    bag_file = "/home/philip/test_scan_bag.bag"
    
    # Topics to record
    topics = ["/scan"]
    
    # Initialize the bag
    bag = rosbag.Bag(bag_file, "w")
    
    # Subscribe to the color image topic
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    
    # Start recording the bag
    rospy.loginfo("Recording bag...")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
    # Stop recording and close the bag
    bag.close()
    rospy.loginfo("Bag recording complete.")

if __name__ == "__main__":
    main()
