#!/usr/bin/env python3

import rospy
from dexterity import Dexterity
from arms import UR5e_Arm
from grippers import GripperNode
from base import BaseNode

NODE_NAME = "live_control"

if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    left_arm = UR5e_Arm(Dexterity.LEFT)
    right_arm = UR5e_Arm(Dexterity.RIGHT)
    left_gripper = GripperNode(Dexterity.LEFT)
    right_gripper = GripperNode(Dexterity.RIGHT)
    husky_base = BaseNode()

