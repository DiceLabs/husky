import sys
import rospy
from arms import UR5e_Arm, Dexterity

END_LINK_JOINT = 5
HALF_RADIAN = .5
NODE_NAME = "TurnGripper"

rospy.init_node(NODE_NAME)
ur_arm = UR5e_Arm(Dexterity.LEFT)
ur_arm.move_joint(END_LINK_JOINT, HALF_RADIAN)