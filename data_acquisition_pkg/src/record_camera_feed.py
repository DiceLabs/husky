#!/usr/bin/env python3

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Global variables
bag = None
bridge = CvBridge()

def color_image_callback(msg):
    global bag, bridge
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Write the image message to the bag
        bag.write("/realsense/color/image_raw", msg, msg.header.stamp)
    except Exception as e:
        rospy.logerr(f"Error converting and recording image: {str(e)}")
def main():
    global bag
    rospy.init_node("realsense_bag_recorder", anonymous=True)
    
    # Path to save the bag file
    bag_file = "/home/philip/test_bag.bag"
    
    # Topics to record
    topics = ["/realsense/color/image_raw"]
    
    # Initialize the bag
    bag = rosbag.Bag(bag_file, "w")
    
    # Subscribe to the color image topic
    rospy.Subscriber("/realsense/color/image_raw", Image, color_image_callback)
    
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
