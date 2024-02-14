import rospy
from math import pi
from arms import UR5e_Arm
from dexterity import Dexterity
from grippers import GripperNode

rospy.init_node(str("set_" + "left" + "_arm"))
left_arm = UR5e_Arm(Dexterity.LEFT)
left_gripper = GripperNode(Dexterity.LEFT)
left_arm.move_joint(2, -3*pi/4) # Move joint to vertical pose
left_arm.move_joint(4, pi/2)    # Point gripper inwards
left_arm.move_joint(5, pi/2)    # Rotate gripper to have fingers move in vertical direction
left_arm.move_vertical(-0.1)             # Move arm up .4 meters
left_arm.move_horizontal(0.1)  # Move arm out to allow space for box
left_arm.move_depth(0.6)                 # Move arm forward .6 meters                
left_gripper.close()
left_arm.move_vertical(1)


rospy.init_node(str("set_" + "right" + "_arm"))
right_arm = UR5e_Arm(Dexterity.RIGHT)
right_gripper = GripperNode(Dexterity.RIGHT)
right_arm.move_joint(2, -3* -1 * pi/4)  # Move joint to vertical pose
right_arm.move_joint(4, -1 * pi/2)      # Point gripper inwards
right_arm.move_joint(5, -1 * pi/2)      # Rotate gripper to have fingers move in vertical direction
right_arm.move_vertical(-0.1)           # Move arm up .4 meters
right_arm.move_horizontal(-0.1)         # Move arm out to allow space for box
right_arm.move_depth(0.6)               # Move arm forward .6 meters    
right_gripper.close()
right_arm.move_vertical(1)
