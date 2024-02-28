#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_ur_arm():
    rospy.init_node('ur_arm_mover', anonymous=True)
    client = actionlib.SimpleActionClient('/right_ur/right_ur_arm_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    goal.trajectory.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0] 
    point.time_from_start = rospy.Duration(5.0) 
    goal.trajectory.points.append(point)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Result: %s", str(result))

if __name__ == '__main__':
    try:
        move_ur_arm()
    except rospy.ROSInterruptException:
        pass
