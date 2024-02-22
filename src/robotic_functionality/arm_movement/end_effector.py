#!/usr/bin/env python3

import rospy
from arms import UR5e_Arm
from dexterity import Dexterity
from robot_types import Euler, Position

rospy.init_node(str("end_effector"))
ur_arm = UR5e_Arm(Dexterity.LEFT)

orientation = Euler(yaw=0, pitch=0, roll=90)
position = Position(x=0,y=0,z=0.0)
ur_arm.move_pose(orientation, position)