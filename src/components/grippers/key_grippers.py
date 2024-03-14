#!/usr/bin/env python3

import rospy
from pynput import keyboard
from grippers import GripperNode
from dexterity import Dexterity

""" 
    @Zix
    This node will listen for key presses which are matched with desired commands of the grippers.
    Uses the ASDF keys for closing and opening of left and right grippers by publishing the correct message to the 
    Gripper action server in real time. 
"""

def on_key_press(key, left_gripper, right_gripper):
    try:
        if key.char == 'c':
            left_gripper.close()
        elif key.char == 'o':
            left_gripper.open()
        elif key.char == 'e':
            right_gripper.open()
        elif key.char == 'p':
            right_gripper.close()
    except AttributeError:
        pass

def print_welcome():
    rospy.loginfo("GRIPPER NODE TURNED ON")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("Welcome to the gripper node! This node will give you control of the robot grippers")
    rospy.loginfo("You can use the keyboard input to give common commands to the grippers")
    rospy.loginfo("o. Open Left Gripper")
    rospy.loginfo("c. Close Left Gripper")
    rospy.loginfo("e. Open Right Gripper")
    rospy.loginfo("n. Close Right Gripper")
    rospy.loginfo("########################################################################################")

def init_node():
    rospy.init_node('GripperNode')
    left_gripper = GripperNode(dexterity=Dexterity.LEFT)
    right_gripper = GripperNode(dexterity=Dexterity.RIGHT)
    print_welcome()
    with keyboard.Listener(
        on_press=lambda key: on_key_press(key, left_gripper, right_gripper)
    ) as listener:
        listener.join()

if __name__ == "__main__" :
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass