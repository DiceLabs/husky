import sys
import rospy
import argparse
from math import pi
from arms import UR5e_Arm
from dexterity import Dexterity
from grippers import GripperNode
    
PI = pi

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

def perform_with_delay(action):
    SLEEP_DURATION = 0.5
    action()
    rospy.sleep(SLEEP_DURATION)

def determine_mirror(arm):
    INVALID_DEXTERITY_MSG = "Invalid Arm Dexterity was Passed"
    if arm.dexterity == Dexterity.LEFT:
        return 1
    elif arm.dexterity == Dexterity.RIGHT:
        return -1
    else:
        exit(INVALID_DEXTERITY_MSG)

def perform_setup_sequence(arm: UR5e_Arm, mirror: bool):
    """ 
        Movements are set to left arm, use mirror variable to perform same action on the right side 
    """
    mirror = determine_mirror(arm)
    perform_with_delay(
        lambda: arm.move_joint(2, mirror * -3*PI/4) # Move joint to vertical pose
    )
    perform_with_delay(
        lambda: arm.move_joint(4, mirror * PI/2)    # Point gripper inwards
    )
    perform_with_delay(
        lambda: arm.move_joint(5, mirror * PI/2)    # Rotate gripper to have fingers move in vertical direction
    )
    perform_with_delay(
        lambda: arm.move_vertical(-0.1)             # Move arm up .4 meters
    )
    perform_with_delay(
        lambda: arm.move_horizontal(mirror * 0.1)  # Move arm out to allow space for box
    )
    perform_with_delay(
        lambda: arm.move_depth(0.6)                 # Move arm forward .6 meters
    )                                       

if __name__ == "__main__":
    DEXTERITY = parse_args(get_args().dexterity)
    rospy.init_node(str("set_" + str(DEXTERITY) + "_arm"))
    ur_arm = UR5e_Arm(DEXTERITY)
    robotiq_gripper = GripperNode(DEXTERITY)
    perform_setup_sequence(ur_arm) # Move arm out to allow space for box
    robotiq_gripper.close_gripper()