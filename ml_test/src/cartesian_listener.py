#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPointStamped

def cartesian_listener():
    rospy.init_node('cartesian_listener', anonymous=True)

    def gps_callback(data):
        # Access the GPS latitude and longitude from the NavSatFix message
        latitude = data.latitude
        longitude = data.longitude

        # Convert GPS coordinates to Cartesian coordinates using UTM
        import utm
        utm_coords = utm.from_latlon(latitude, longitude)
        x, y, zone_number, zone_letter = utm_coords

        rospy.loginfo(f"GPS coordinates: (latitude={latitude}, longitude={longitude})")
        rospy.loginfo(f"Transformed Cartesian coordinates: (x={x}, y={y})")

    rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        cartesian_listener()
    except rospy.ROSInterruptException:
        pass
