#!/usr/bin/env python3

import rospy
from pynput import keyboard
from arms import robot_arm

""" 
    @Zix
    This node will listen for key presses which are matched with desired commands of the base.
    Uses the ASDF keys for the desired movements by publishing the correct message to the 
    base action server in real time. 
"""

def on_key_press(key, arm_node: robot_arm):
    key_actions = {
        'u': arm_node.move_up,
        'd': arm_node.move_down,
        'f': arm_node.move_forward,
        'b': arm_node.move_backward,
        'l': arm_node.move_left,
        'r': arm_node.move_right,
    }
    action = key_actions.get(key.char)
    if action is not None:
        action()

def print_welcome():
    rospy.loginfo("ARM NODE TURNED ON")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("Welcome to the base node! This node will give you control of the robot bases")
    rospy.loginfo("You can use the keyboard input to give common commands to the bases")
    rospy.loginfo("'u': move_up,")
    rospy.loginfo("'d': move_down,")
    rospy.loginfo("'f': move_forward,")
    rospy.loginfo("'b': move_backward,")
    rospy.loginfo("'l': move_left,")
    rospy.loginfo("'r': move_right,")
    rospy.loginfo("########################################################################################")

def init_node():
    rospy.init_node('ArmNode')
    arm_node = robot_arm()
    arm_node.set_initial_arm_params()
    print_welcome()
    with keyboard.Listener(
        on_press=lambda key: on_key_press(key, arm_node)
    ) as listener:
        listener.join()

if __name__ == "__main__" :
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass