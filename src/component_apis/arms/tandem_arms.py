import rospy
from arms import UR5e_Arm
from dexterity import Dexterity
from grippers import GripperNode
from robot_types import Euler, Position

def lift_sequence():
    left_arm.move_up()
    right_arm.move_up()
    left_arm.move_up()
    right_arm.move_up()
    left_arm.move_up()
    right_arm.move_up()

if __name__ == "__main__":
    rospy.init_node(str("tandem_arms"))
    left_arm = UR5e_Arm(Dexterity.LEFT)
    left_gripper = GripperNode(Dexterity.LEFT)
    right_arm = UR5e_Arm(Dexterity.RIGHT)
    right_gripper = GripperNode(Dexterity.RIGHT)

    left_arm.change_pose(orientation=Euler(), position=Position(x=0.5, y=-0.3, z=-0.5))
    right_arm.change_pose(orientation=Euler(), position=Position(x=0.5, y=0.3, z=-0.5))
