#!/usr/bin/python3
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2

rospy.init_node("view_objects", anonymous=True)


camera_color_image_topic = rospy.get_param("~camera_color_image_topic")
YOLO_model = rospy.get_param("~YOLO_model")
model = YOLO(YOLO_model)
bridge = CvBridge()

def callback0(msg):
	cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  # convert to
	result = model(cv_image, verbose=True)[0]
	annotated_frame = result.plot()
	cv2.imshow("YOLOv5 Inference", annotated_frame)
	cv2.waitKey(1)

if __name__ == "__main__":
	try:
		rospy.Subscriber(camera_color_image_topic, Image, callback0, queue_size=10, buff_size=2_000_000)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
