import rospy
from arms import UR5e_Arm, Dexterity
import multiprocessing
from math import pi

NPI = -1 * pi
PI = pi

END_LINK_JOINT = 5
HALF_RADIAN = .5
NODE_NAME = "TurnGripper"
rospy.init_node(NODE_NAME)

left_arm = UR5e_Arm(Dexterity.LEFT)
right_arm = UR5e_Arm(Dexterity.RIGHT)

def position_left():
    left_arm.move_joint(2, 3*NPI/4) # Move Arm To vertical position
    left_arm.move_joint(4, PI/2)    # Point gripper inwards
    left_arm.move_joint(5, PI/2)    # Rotate gripper to have fingers move in vertical direction
    left_arm.move_depth(.6)         # Move arm forward .6 meters
    left_arm.move_vertical(.4)      # Move arm up .4 meters

def position_right():
    right_arm.move_joint(2,  3*PI/4)  # Move Arm To vertical position
    right_arm.move_joint(4, -PI/2)    # Point gripper inwards
    right_arm.move_joint(5, -PI/2)    # Rotate gripper to have fingers move in vertical direction
    right_arm.move_depth(.6)          # Move arm forward .6 meters         
    right_arm.move_vertical(.4)       # Move arm up .4 meters


process_left = multiprocessing.Process(target=position_left)
process_right = multiprocessing.Process(target=position_right)

process_left.start()
process_right.start()

process_left.join()
process_right.join()
