import rospy
from math import pi
from arms import UR5e_Arm
from dexterity import Dexterity
from grippers import GripperNode

rospy.init_node(str("set_" + "left" + "_arm"))
left_arm = UR5e_Arm(Dexterity.LEFT)
left_gripper = GripperNode(Dexterity.LEFT)

def move_left():
    rospy.sleep(0.05)
    left_arm.move_joint(2, -3*pi/4) # Move joint to vertical pose
    rospy.sleep(0.05)
    left_arm.move_joint(4, pi/2)    # Point gripper inwards
    rospy.sleep(0.05)
    left_arm.move_joint(5, pi/2)    # Rotate gripper to have fingers move in vertical direction
    rospy.sleep(0.05)
    left_arm.move_horizontal(0.1)  # Move arm out to allow space for box
    rospy.sleep(0.05)
    left_arm.move_vertical(-0.1)             # Move arm up .4 meters
    rospy.sleep(0.05)
    left_arm.move_depth(0.6)                 # Move arm forward .6 meters 
    rospy.sleep(0.05)              
    left_arm.move_horizontal(-0.115)  # Move arm out to allow space for box
    rospy.sleep(0.05)

left_gripper.close()
rospy.sleep(0.05)
left_arm.move_vertical(1)

rospy.init_node(str("set_" + "right" + "_arm"))
right_arm = UR5e_Arm(Dexterity.RIGHT)
right_gripper = GripperNode(Dexterity.RIGHT)

def move_right():
    rospy.sleep(0.05)
    right_arm.move_joint(2, -3* -1 * pi/4)  # Move joint to vertical pose
    rospy.sleep(0.05)
    right_arm.move_joint(4, -1 * pi/2)      # Point gripper inwards
    rospy.sleep(0.05)
    right_arm.move_joint(5, -1 * pi/2)      # Rotate gripper to have fingers move in vertical direction
    rospy.sleep(0.05)
    right_arm.move_horizontal(-0.1)         # Move arm out to allow space for box
    rospy.sleep(0.05)
    right_arm.move_vertical(-0.1)           # Move arm up .4 meters
    rospy.sleep(0.05)
    right_arm.move_depth(0.6)               # Move arm forward .6 meters    
    rospy.sleep(0.05)
    right_arm.move_horizontal(0.115)  


def lift_left():
    for _ in range(1, 6):
        left_arm.move_vertical(.01)
        rospy.sleep(.5)

def lift_right():
    for _ in range(1, 6):
        rospy.sleep(.5)
        right_arm.move_vertical(.01)


right_gripper.close()
rospy.sleep(0.01)
right_arm.move_vertical(1)


left_gripper.close()
right_gripper.close()

left_gripper.open()
right_gripper.open()