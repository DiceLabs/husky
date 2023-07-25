#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def gps_publisher():
    rospy.init_node('gps_publisher', anonymous=True)
    rate = rospy.Rate(1)  # Publish data at 1 Hz

    gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

    while not rospy.is_shutdown():
        # Simulated GPS data (replace with your desired latitude and longitude)
        latitude = 40.7128  # New York City
        longitude = -74.0060

        gps_msg = NavSatFix()
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = 0.0  # Assuming 0 altitude for simplicity

        gps_pub.publish(gps_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
