#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from generate_point import arnold_RK4
import tf
from tf.transformations import quaternion_from_euler
import math

# Initialize ROS node
rospy.init_node('move_to_target_position', anonymous=True)

def move_to_target_position(target_position):
    angle = -0.145496365487901
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi

    yaw = angle  # yaw value in the range of -pi to pi
    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    # Create action client for move_base
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    # Create a goal object
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = target_position[0]
    goal.target_pose.pose.position.y = target_position[1]
    goal.target_pose.pose.orientation.x =  quaternion[0]
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


def move_it():
	#listener = tf.TransformListener()
	#listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
	#(trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
	#arnpnt_0 = arnold_RK4(0.5, 0.25, 0.25, 0.22, 0, 1, 0,  trans[0], trans[1], 10.5, 2)
	move_to_target_position([3.315422039176631, -6.420113263857609])

if __name__ == '__main__':
	try:
		# Specify the target position [x, y]
		move_it()
	except rospy.ROSInterruptException:
		pass

