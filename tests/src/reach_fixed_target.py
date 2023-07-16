#!/usr/bin/python3
import rospy
import moveit_commander
import sys
import geometry_msgs.msg
def move_ur_to_target():
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur_node', anonymous=True)

    # Instantiate a MoveGroupCommander for the UR arm group
    arm_group = moveit_commander.MoveGroupCommander("manipulator_left")

    # Set the target position and orientation
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 1.2  # Set your desired target position
    target_pose.position.y = 0.3
    target_pose.position.z = 0.2
    # Set your desired target orientation if needed

    # Plan and execute the motion
    arm_group.set_pose_target(target_pose)
    arm_group.go(wait=True)

    # Shutdown MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_ur_to_target()
    except rospy.ROSInterruptException:
        pass

