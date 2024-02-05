#!/usr/bin/env python3

import sys
import rospy
import argparse
from pynput import keyboard
from arms import UR5e_Arm, Dexterity

""" 
    @Zix
    This node will listen for key presses which are matched with desired commands of the base.
    Uses the ASDF keys for the desired movements by publishing the correct message to the 
    base action server in real time. 
"""

def print_welcome(arm_dexterity):
    rospy.loginfo("ARM NODE TURNED ON WITH DEXTERITY %s" % str(arm_dexterity).upper())
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("Welcome to the base node! This node will give you control of the robot bases")
    rospy.loginfo("You can use the keyboard input to give common commands to the bases")
    rospy.loginfo("q: left_arm.move_up")
    rospy.loginfo("w: left_arm.move_down")
    rospy.loginfo("a: left_arm.move_forward")
    rospy.loginfo("s: left_arm.move_backward")
    rospy.loginfo("d: left_arm.move_left")
    rospy.loginfo("f: left_arm.move_right")
    rospy.loginfo("h: right_arm.move_up")
    rospy.loginfo("j: right_arm.move_down")
    rospy.loginfo("u: right_arm.move_forward")
    rospy.loginfo("i: right_arm.move_backward")
    rospy.loginfo("k: right_arm.move_left")
    rospy.loginfo("l: right_arm.move_right")
    rospy.loginfo("########################################################################################")


def on_key_press(key, ur5e_arm, actions):
    KEY_FAIL_MSG = "Key Press Failed, Continuing"

    action_map = actions.get(ur5e_arm.dexterity, {})
    try:
        action = None
        if hasattr(key, 'char') and key.char is not None:
            action = action_map.get(key.char)
        if action:
            action()
    except rospy.ROSInterruptException:
        print(KEY_FAIL_MSG)

def get_args():
    FLAG_ID = '-d'
    FLAG_ATTR = '--dexterity'
    FLAG_HELP = "indicate dexterity of ur5e_arm with d flag followed by l for left and r for right"
    NO_FLAG_MSG = "No command line argument passed. Please provide an argument."
    MIN_ARG_COUNT = 2

    if len(sys.argv) < MIN_ARG_COUNT:
        raise Exception(NO_FLAG_MSG)
    parser = argparse.ArgumentParser(description=FLAG_HELP)
    parser.add_argument(FLAG_ID, FLAG_ATTR)
    return parser.parse_args()

def parse_args(value): # Expected to be "l" or "r"
    LEFT_FLAG = "l"
    RIGHT_FLAG = "r"
    INVALID_DEXT_ARG_MSG = "Invalid value for dexterity. Use 'l' for left or 'r' for right."
    
    flag_converter = {LEFT_FLAG: Dexterity.LEFT, RIGHT_FLAG: Dexterity.RIGHT}
    if value.lower() in flag_converter:
        return flag_converter[value.lower()]
    else:
        raise argparse.ArgumentTypeError(INVALID_DEXT_ARG_MSG)

def define_actions(arm):
    return {
        Dexterity.LEFT: {
            'q': arm.move_up,
            'w': arm.move_down,
            'a': arm.move_forward,
            's': arm.move_backward,
            'd': arm.move_left,
            'f': arm.move_right,
        },
        Dexterity.RIGHT: {
            'h': arm.move_up,
            'j': arm.move_down,
            'u': arm.move_forward,
            'i': arm.move_backward,
            'k': arm.move_left,
            'l': arm.move_right,
        }
    }

def init_node():
    DEXTERITY = parse_args(get_args().dexterity)
    rospy.init_node(str(DEXTERITY))
    print_welcome(DEXTERITY)
    ur_arm = UR5e_Arm(DEXTERITY)
    actions = define_actions(ur_arm)
    with keyboard.Listener(
        on_press=lambda key: on_key_press(key, ur_arm, actions)
    ) as listener:
        listener.join()

if __name__ == "__main__" :
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass