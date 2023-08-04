#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf
from rospy.numpy_msg import numpy_msg
import numpy as np

from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler
import math
from nav_msgs.msg import Odometry
import math
# Initialize ROS node
rospy.init_node('robot_navigation_to_object', anonymous=True)


class converter:
	def __init__(self):
		self.a_3D_point = None
		self.rot=None
		self.camera_frame = rospy.get_param("/robot_navigation_to_object/camera_depth_optical_frame")
		self.map_frame = rospy.get_param("/robot_navigation_to_object/map_frame")
		self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
		self.goal = PoseStamped()
		self.odom = Odometry()
		self.loc = None

	def transform(self,coord, mat44):
		print("MAT44: ",mat44)
		print("POINT ", coord)
		xyz = tuple(np.dot(mat44, np.array([coord[0], coord[1], coord[2], 1.0])))[:3]
		r = geometry_msgs.msg.PointStamped()
		r.point = geometry_msgs.msg.Point(*xyz)
		return [r.point.x, r.point.y, r.point.z]

	def move_it(self):
		listener = tf.TransformListener()
		#secondary_camera_realsense_depth_optical_frame
		"""Get transform matrix to convert Point in 3D space to from camera_realsense_link_gazebo to base_link"""
		listener.waitForTransform(self.map_frame, self.camera_frame, rospy.Time(0), rospy.Duration(10.0))
		(trans, rot) = listener.lookupTransform(self.map_frame, self.camera_frame, rospy.Time(0))
		TF_c_b = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

		listener.waitForTransform('odom', self.camera_frame, rospy.Time(0), rospy.Duration(10.0))
		(trans, rot) = listener.lookupTransform('odom', self.camera_frame, rospy.Time(0))
		TF_odom = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

		#_, _, yaw = tf.transformations.euler_from_matrix(TF_c_b, axes='sxyz')
		##self.a_3D_point[3] + yaw

		point = self.transform(self.a_3D_point ,TF_c_b)

		self.loc = self.transform(self.a_3D_point, TF_odom)

		x = point[0]
		y = point[1]
		# Determine the quadrant
		quadrant = None
		if x > 0 and y > 0:
			quadrant = "Quadrant I"
		elif x < 0 and y > 0:
			quadrant = "Quadrant II"
		elif x < 0 and y < 0:
			quadrant = "Quadrant III"
		elif x > 0 and y < 0:
			quadrant = "Quadrant IV"
		elif x == 0 and y != 0:
			quadrant = "On the y-axis"
		elif x != 0 and y == 0:
			quadrant = "On the x-axis"
		else:
			quadrant = "At the origin"

		# Relate quadrants to the map frame axes
		if quadrant == "Quadrant I":
			print(quadrant)
			self.rot = math.atan2(abs(point[1]), abs(point[0]));
		elif quadrant == "Quadrant II":
			print(quadrant)
			self.rot = (math.pi/2) + math.atan2(abs(point[0]), abs(point[1]))
		elif quadrant == "Quadrant III":
			print(quadrant)
			self.rot = -(math.pi/2) - math.atan2(abs(point[0]), abs(point[1]))
		elif quadrant == "Quadrant IV":
			print(quadrant)
			self.rot =  -math.atan2(abs(point[1]), abs(point[0]))
		elif quadrant == "On the x-axis" :
			if x > 0:
				self.rot = 0.0

			if x < 0:
				self.rot = math.pi
		elif quadrant == "On the y-axis":
			if y > 0:
				self.rot = math.pi/2

			if y < 0:
				self.rot = -math.pi/2
		else:
			self.rot = 0.0

		print("POINT: ",point)
		print("angle: ", self.rot)
		self.move_to_target_position(point)

	def callback(self,msg):
		self.a_3D_point = msg.data
		self.move_it()

	def callback2(self,msg):
		self.odom = msg


	def move_to_target_position(self,target_position):
		quaternion = quaternion_from_euler(0.0, 0.0, self.rot)

		self.goal.header.stamp = rospy.get_rostime()
		self.goal.header.frame_id = 'map'
		self.goal.pose.position.x = target_position[0]
		self.goal.pose.position.y = target_position[1]
		self.goal.pose.position.z = 0
		self.goal.pose.orientation.x = quaternion[0]
		self.goal.pose.orientation.y = quaternion[1]
		self.goal.pose.orientation.z = quaternion[2]
		self.goal.pose.orientation.w = quaternion[3]
		x_loc = self.odom.pose.pose.position.x
		y_loc = self.odom.pose.pose.position.y
		reach = math.sqrt((self.goal.pose.position.x - x_loc) ** 2 + (self.goal.pose.position.y  - y_loc) ** 2)
		while reach >= 0.5:
			if reach == 0.2:
				break
			self.pub.publish(self.goal)
			rospy.sleep(0.5)
			x_loc = self.odom.pose.pose.position.x
			y_loc = self.odom.pose.pose.position.y
			reach = math.sqrt((self.goal.pose.position.x - x_loc) ** 2 + (self.goal.pose.position.y  - y_loc) ** 2)

def main():
	test  = converter()
	rospy.Subscriber("/odometry/filtered", Odometry, test.callback2, queue_size=1)
	rospy.Subscriber("/the_points", Float64MultiArray, test.callback, queue_size=1)

	rospy.spin()



if __name__ == "__main__":
	main()
