#!/usr/bin/env python3

import rospy
from pynput import keyboard
from grippers import open_gripper, close_gripper, GripperNode

""" 
    @Zix
    This node will listen for key presses which are matched with desired commands of the grippers.
    Uses the ASDF keys for closing and opening of left and right grippers by publishing the correct message to the 
    Gripper action server in real time. 
"""

def on_key_press(key, gripper_node):
    try:
        if key.char == 'c':
            close_gripper(gripper_node.left_pub)
        elif key.char == 'o':
            open_gripper(gripper_node.left_pub)
        elif key.char == 'e':
            close_gripper(gripper_node.right_pub)
        elif key.char == 'p':
            open_gripper(gripper_node.right_pub)
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
    gripper_node = GripperNode()
    print_welcome()
    with keyboard.Listener(
        on_press=lambda key: on_key_press(key, gripper_node)
    ) as listener:
        listener.join()

if __name__ == "__main__" :
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass