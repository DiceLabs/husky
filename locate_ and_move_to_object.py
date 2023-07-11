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

# Initialize ROS node
rospy.init_node('cam_to_map', anonymous=True)

class converter:
	def __init__(self):
		self.a_3D_point = None
		self.rot=None


	def transform(self,coord, mat44):
		print("MAT44: ",mat44)
		print("POINT ", coord)
		xyz = tuple(np.dot(mat44, np.array([coord[0], coord[1], coord[2], 1.0])))[:3]
		r = geometry_msgs.msg.PointStamped()
		r.point = geometry_msgs.msg.Point(*xyz)
		return [r.point.x, r.point.y, r.point.z]


	def move_it(self):
		listener = tf.TransformListener()
		"""Get transform matrix to convert Point in 3D space to from camera_realsense_link_gazebo to base_link"""
		listener.waitForTransform('map', 'camera_realsense_link_gazebo', rospy.Time(0), rospy.Duration(10.0))
		(trans, rot) = listener.lookupTransform('map', 'camera_realsense_link_gazebo', rospy.Time(0))
		TF_c_b = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

		#_, _, yaw = tf.transformations.euler_from_matrix(TF_c_b, axes='sxyz')
		##self.a_3D_point[3] + yaw

		point = self.transform(self.a_3D_point ,TF_c_b)
		#self.rot = point[0]-trans[0],point[1]-trans[1]

		print("PRE: ",self.rot)
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
			self.rot =  -((math.pi/2)-math.atan2(abs(point[0]), abs(point[1])))
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

	def move_to_target_position(self,target_position):
		#angle = self.rot #self.a_3D_point[3] * -1
		#if angle > math.pi:
		#	angle -= 2 * math.pi
		#elif angle < -math.pi:
		#	angle += 2 * math.pi

		#yaw = angle # yaw value in the range of -pi to pi
		quaternion = quaternion_from_euler(0.0, 0.0, self.rot)
		# Create action client for move_base
		move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		move_base_client.wait_for_server()

		# Create a goal object
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = target_position[0]
		goal.target_pose.pose.position.y = target_position[1]
		goal.target_pose.pose.orientation.x = quaternion[0]
		goal.target_pose.pose.orientation.y = quaternion[1]
		goal.target_pose.pose.orientation.z = quaternion[2]
		goal.target_pose.pose.orientation.w = quaternion[3]

		# Send the goal to the action server
		move_base_client.send_goal(goal)

		# Wait for the robot to reach the goal or for it to be aborted
		move_base_client.wait_for_result()

		# Check the result of the navigation
		if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
			rospy.loginfo('Goal reached successfully.')
		else:
			rospy.loginfo('Failed to reach the goal.')
def main():
	test  = converter()
	rospy.Subscriber("/the_points", Float64MultiArray, test.callback, queue_size=1)

	rospy.spin()



if __name__ == "__main__":
	main()
