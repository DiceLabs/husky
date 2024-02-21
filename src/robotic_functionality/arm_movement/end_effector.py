#!/usr/bin/env python3

import rospy
from math import pi
from arms import UR5e_Arm
from grippers import GripperNode
from dexterity import Dexterity
from robot_types import Euler, Position

def give_pose(arm: UR5e_Arm):
    orientation = Euler(yaw=0, pitch=0, roll=90)
    position = Position(x=0.6, y=0.3, z=-0.7)
    arm.pose_goal(orientation, position)        

def go_back(arm: UR5e_Arm):
    orientation = Euler(yaw=0, pitch=0, roll=-90)
    position = Position(x=-0.6, y=-0.3, z=0.7)
    arm.pose_goal(orientation, position)  

if __name__ == "__main__":
    rospy.init_node(str("end_effector"))
    ur_arm = UR5e_Arm(Dexterity.LEFT)
    robotiq_gripper = GripperNode(Dexterity.LEFT)
    give_pose(ur_arm)
