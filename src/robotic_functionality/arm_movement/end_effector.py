#!/usr/bin/env python3

import rospy
from arms import UR5e_Arm
from dexterity import Dexterity
from robot_types import Euler, Position, Quaternion
from typing import Callable

rospy.init_node(str("end_effector"))
left_arm = UR5e_Arm(Dexterity.LEFT)
right_arm = UR5e_Arm(Dexterity.LEFT)

def example_pose():
    left_arm.change_pose(orientation=Euler(roll=20), position=Position())
    right_arm.change_pose(orientation=Euler(roll=20), position=Position())

def reset_pose():
    left_arm.change_pose_abs (orientation=Quaternion(w=.5, x=.5, y=.5, z=.5), position=Position(x=0.5, y=-0.5, z=1))
    right_arm.change_pose_abs(orientation=Quaternion(w=.5, x=.5, y=.5, z=.5), position=Position(x=0.5, y= 0.5, z=1))

def perform_action(action: Callable, times: int):
    i = times
    while i > 0:
        action()
        i -= 1

def move_up(left_arm: UR5e_Arm, right_arm: UR5e_Arm):
    left_arm.move_up()
    right_arm.move_up()

def lift(left_arm: UR5e_Arm, right_arm: UR5e_Arm, distance: float):
    intervals = distance / 0.1
    perform_action(lambda: move_up(left_arm=left_arm, right_arm=right_arm), intervals)

