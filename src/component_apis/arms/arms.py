#!/usr/bin/python3

import sys
import rospy
from enum import Enum
import moveit_commander
from geometry_msgs.msg import Pose
from robot_types import Position, Orientation

NODE_NAME = 'moveit_arm_api'
END_EFFECTOR_SUFFIX = "_ur_arm_wrist_3_link"
MANIPULATOR_PREFIX = "manipulator_"

VELOCITY_SCALING_CONSTANT = 0.1

class Dexterity(Enum):
    LEFT = 0,
    RIGHT = 1
    def __str__(self):
        return self.name.lower()

class UR5e_Arm:
    """ 
        Arm can be initialized as left or right. Connects to moveit client and can take advantage of various functions of moveit API,
        notably the ability to set joint targets or a pose target, other functions have been included to ease the use of common movements 
    """
    def __init__(self, dexterity: Dexterity):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(MANIPULATOR_PREFIX + str(dexterity))
        self.dexterity = dexterity
    def print_info(self):
        print ("============ Reference frame: %s" % self.group.get_planning_frame())
        print ("============ End effector: %s" % self.group.get_end_effector_link())
        print ("============ Current State: %s" % self.group.get_current_state())
        print ("============ Robot Groups:", self.robot.get_group_names())
    def move_joint(self, joint_id: int, amount: float):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[joint_id] += amount
        self.group.go(joint_goal, wait=True)
        self.group.stop()
    def move_pose(self, orientation: Orientation, position: Position):
        pose_goal = self.create_pose_goal(orientation, position) 
        self.group.set_pose_target(pose_goal, str(self.dexterity) + END_EFFECTOR_SUFFIX)
        self.group.set_max_velocity_scaling_factor(VELOCITY_SCALING_CONSTANT)
        self.group.go(wait=True)
        self.group.stop()
    def create_pose_goal(self, orientation: Orientation, position: Position):
        pose_goal = Pose()
        current = self.group.get_current_pose().pose
        pose_goal.orientation.w = current.orientation.w + orientation.w
        pose_goal.orientation.x = current.orientation.x + orientation.x
        pose_goal.orientation.y = current.orientation.y + orientation.y
        pose_goal.orientation.z = current.orientation.z + orientation.z
        pose_goal.position.x = current.position.x + position.x
        pose_goal.position.y = current.position.y + position.y
        pose_goal.position.z = current.position.z + position.z
        return pose_goal

    def move_vertical(self, amount: float):
        position_delta = Position(z=amount)
        orientation_delta = Orientation()
        self.move_pose(orientation_delta, position_delta)
    def move_horizontal(self, amount: float):
        position_delta = Position(y=amount)
        orientation_delta = Orientation()
        self.move_pose(orientation_delta, position_delta)
    def move_depth(self, amount: float):
        position_delta = Position(x=amount)
        orientation_delta = Orientation()
        self.move_pose(orientation_delta, position_delta)

    def yaw(self, amount):
        position_delta = Position()
        orientation_delta = Orientation()
        self.move_pose(orientation_delta, position_delta)
    def pitch(self, amount):
        position_delta = Position()
        orientation_delta = Orientation()
        self.move_pose(orientation_delta, position_delta)
    def roll(self, amount):
        position_delta = Position()
        orientation_delta = Orientation()
        self.move_pose(orientation_delta, position_delta)

    def move_up(self):
        self.move_vertical(0.1)
    def move_down(self):
        self.move_vertical(-0.1)
    def move_left(self):
        self.move_horizontal(0.1)
    def move_right(self):
        self.move_horizontal(-0.1)
    def move_forward(self):
        self.move_depth(0.1)
    def move_backward(self):
        self.move_depth(-0.1)

if __name__ == "__main__" :
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(NODE_NAME, anonymous=True)
    left_arm = UR5e_Arm(Dexterity.LEFT)
    right_arm = UR5e_Arm(Dexterity.RIGHT)
    left_arm.print_info()
    right_arm.print_info()
