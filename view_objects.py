#!/usr/bin/python3
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2

model = YOLO('yolov5lu.pt')
bridge = CvBridge()

def callback0(msg):
	cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # convert to
	result = model(cv_image, verbose=True)[0]
	annotated_frame = result.plot()
	cv2.imshow("YOLOv5 Inference", annotated_frame)
	cv2.waitKey(1)

if __name__ == "__main__":
	try:
		rospy.init_node("test_node_2")
		rospy.Subscriber("/realsense/color/image_raw", Image, callback0, queue_size=10, buff_size=2_000_000)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
