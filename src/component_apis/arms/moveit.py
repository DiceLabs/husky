#!/usr/bin/python3

import rospy
import sys
import moveit_commander

NODE_NAME = 'moveit_arm_api'
LEFT_ARM = "manipulator_left"
RIGHT_ARM = "manipulator_right"

class UR5e_Arm:
    def __init__(self, manipulator):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(manipulator)
        self.print_info()
    def print_info(self):
        print ("============ Reference frame: %s" % self.group.get_planning_frame())
        print ("============ End effector: %s" % self.group.get_end_effector_link())
        print ("============ Current State: %s" % self.group.get_current_state())
        print ("============ Robot Groups:", self.robot.get_group_names())
        print ("============ Robot state: %s", self.robot.get_current_state())
    def move_joint_zero(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] += .2
        self.group.go(joint_goal, wait=True)
        self.group.stop()

if __name__ == "__main__" :
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(NODE_NAME, anonymous=True)
    left_arm = UR5e_Arm(LEFT_ARM)
    right_arm = UR5e_Arm(RIGHT_ARM)
    left_arm.move_joint_zero()
    right_arm.move_joint_zero()


""" 
    def execute(self):
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
    def setPose(self):
        pose_goal = Pose()
        self.group.set_pose_target(pose_goal)
"""