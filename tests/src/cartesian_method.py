#!/usr/bin/python3
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import numpy as np
import rostopic
from rosgraph_msgs.msg import Log

rospy.init_node('move_group_example', anonymous=True)

roslog = Log()

def log_callback(log_msg):
    global roslog
    roslog=log_msg
        
# Get the topic name for /rosout
log_topic = '/rosout'

# Subscribe to the log topic and set the callback
rospy.Subscriber(log_topic, Log, log_callback)


moveit_commander.roscpp_initialize(sys.argv)

        
# Instantiate a MoveGroupCommander object
group_name = "manipulator_right"  # Replace with your planning group name
move_group = moveit_commander.MoveGroupCommander(group_name)

# Set the reference frame for pose targets
reference_frame = "base_link"  # Replace with your desired reference frame
move_group.set_pose_reference_frame(reference_frame)

# Set the end-effector link
end_effector_link = move_group.get_end_effector_link()

# Set the target pose using a geometry_msgs.msg.Pose object
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = 0.3  # Set your desired X coordinate
target_pose.position.y = -0.2 # Set your desired Y coordinate
target_pose.position.z = 0.5  # Set your desired Z coordinate
target_pose.orientation.x = 0.0  # Set your desired orientation (Quaternion)
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 1.0

move_group.set_max_velocity_scaling_factor(0.1)

# Get the JointModelGroup for the planning group
joint_model_group = move_group.get_joints()
print(joint_model_group)

# Set joint limits for each joint in the JointModelGroup
#joint_names = joint_model_group.get_active_joint_names()
	
current_joint_angles = move_group.get_current_joint_values()

# Set a new planning timeout (in seconds)
new_timeout = 3.0  # Adjust the timeout value as needed

# Update the planning timeout
move_group.set_planning_time(new_timeout)


# Use compute_cartesian_path to generate a Cartesian path to the target pose
(cartesian_path,fraction) = move_group.compute_cartesian_path(
    [target_pose],     # List of waypoints
    0.01,               # Step size (distance between waypoints in Cartesian space)
    0.0,                # Jump threshold (0 means no jump allowed between waypoints)
    avoid_collisions=True)  # Collision avoidance during the path planning

# If the path planning is successful, execute the Cartesian path
if cartesian_path:
    #move_group.set_goal_orientation_tolerance(2*np.pi)
    move_group.execute(cartesian_path, wait=True)
    #global roslog
    if roslog.msg == 'Execution completed: ABORTED':
        print(roslog.msg)
        move_group.go(current_joint_angles, wait=True)
        print("hurray")
    	
    else:
        rospy.loginfo("Motion planning and execution succeeded!")
        #move_group.go(current_joint_angles, wait=True)
        current_joint_angles = move_group.get_current_joint_values()

else:
    rospy.logerr("Cartesian path planning failed!")
    
rospy.spin()

