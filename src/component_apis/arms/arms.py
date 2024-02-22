#!/usr/bin/python3

import sys
import rospy
import moveit_commander
from dexterity import Dexterity
from geometry_msgs.msg import Pose
from robot_types import Position, Quaternion, Euler
from transformations import quaternion_multiply, quaternion_from_euler
from conversions import degrees_to_radians

NODE_NAME = 'moveit_arm_api'
END_EFFECTOR_SUFFIX = "_ur_arm_wrist_3_link"
MANIPULATOR_PREFIX = "manipulator_"

VELOCITY_SCALING_CONSTANT = 0.5
ZERO = 0

class UR5e_Arm:
    """ 
        Arm can be initialized as left or right. Connects to moveit client and can take advantage of various functions of moveit API,
        notably the ability to set joint targets or a pose target, other functions have been included to ease the use of common movements 
    """
    def __init__(self, dexterity: Dexterity):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(MANIPULATOR_PREFIX + str(dexterity))
        self.group.set_max_velocity_scaling_factor(VELOCITY_SCALING_CONSTANT)
        self.dexterity = dexterity
    def print_info(self):
        print ("============ Reference frame: %s" % self.group.get_planning_frame())
        print ("============ End effector: %s" % self.group.get_end_effector_link())
        print ("============ Current State: %s" % self.group.get_current_state())
        print ("============ Robot Groups:", self.robot.get_group_names())
    def move_joint(self, joint_id: int, amount: float):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[joint_id] += amount
        self.group.go(joint_goal, wait=False)
        self.group.stop()
    def move_pose(self, orientation: Euler, position: Position):
        yaw, pitch, roll = degrees_to_radians(yaw=orientation.yaw, pitch=orientation.pitch, roll=orientation.roll)
        degree_orientation = Euler(yaw=yaw, pitch=pitch, roll=roll)
        pose_goal = self.create_pose_goal(degree_orientation, position) 
        self.group.set_pose_target(pose_goal, str(self.dexterity) + END_EFFECTOR_SUFFIX)
        self.group.set_max_velocity_scaling_factor(VELOCITY_SCALING_CONSTANT)
        self.group.go(wait=False)
        self.group.stop()
    def create_pose_goal(self, orientation: Euler, position: Position):
        pose_goal = Pose()
        current = self.group.get_current_pose().pose
        delta_orientation_quat = quaternion_from_euler(orientation.yaw, orientation.pitch, orientation.roll)
        current_orientation_quat = [current.orientation.w, current.orientation.x, current.orientation.y, current.orientation.z]
        new_orientation_quat = quaternion_multiply(current_orientation_quat, delta_orientation_quat)
        pose_goal.orientation.w = new_orientation_quat[0]
        pose_goal.orientation.x = new_orientation_quat[1]
        pose_goal.orientation.y = new_orientation_quat[2]
        pose_goal.orientation.z = new_orientation_quat[3]
        pose_goal.position.x = current.position.x + position.x
        pose_goal.position.y = current.position.y + position.y
        pose_goal.position.z = current.position.z + position.z
        return pose_goal

    def move_vertical(self, amount: float):
        position_delta = Position(z=amount)
        orientation_delta = Euler()
        self.move_pose(orientation_delta, position_delta)
    def move_horizontal(self, amount: float):
        position_delta = Position(y=amount)
        orientation_delta = Euler()
        self.move_pose(orientation_delta, position_delta)
    def move_depth(self, amount: float):
        position_delta = Position(x=amount)
        orientation_delta = Euler()
        self.move_pose(orientation_delta, position_delta)

    def yaw(self, amount):
        position_delta = Position()
        yaw, pitch, roll = degrees_to_radians(yaw=amount, pitch=ZERO, roll=ZERO)
        orientation_delta = Euler(yaw=yaw, pitch=pitch, roll=roll)
        self.move_pose(orientation_delta, position_delta)
    def pitch(self, amount):
        position_delta = Position()
        yaw, pitch, roll = degrees_to_radians(yaw=ZERO, pitch=amount, roll=ZERO)
        orientation_delta = Euler(yaw=yaw, pitch=pitch, roll=roll)
        self.move_pose(orientation_delta, position_delta)
    def roll(self, amount):
        position_delta = Position()
        yaw, pitch, roll = degrees_to_radians(yaw=ZERO, pitch=ZERO, roll=amount)
        orientation_delta = Euler(yaw=yaw, pitch=pitch, roll=roll)
        self.move_pose(orientation_delta, position_delta)
    def roll_zero(self):
        DESIRED_ROLL = ZERO
        position_delta = Position()
        yaw, pitch, roll = degrees_to_radians(yaw=ZERO, pitch=ZERO, roll=DESIRED_ROLL)
        orientation_delta = Euler(yaw=yaw, pitch=pitch, roll=roll)
        self.move_pose(orientation_delta, position_delta)
    def roll_ninety(self):
        DESIRED_ROLL = 90
        position_delta = Position()
        yaw, pitch, roll = degrees_to_radians(yaw=ZERO, pitch=ZERO, roll=DESIRED_ROLL)
        orientation_delta = Euler(yaw=yaw, pitch=pitch, roll=roll)
        self.move_pose(orientation_delta, position_delta)

    def go_back(self):
        last_command = []
        

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


""" 
    BOX Sequence

    left_arm.move_joint(2, 3*NPI/4) # Move Arm To vertical position
    left_arm.move_joint(4, PI/2)    # Point gripper inwards
    left_arm.move_joint(5, PI/2)    # Rotate gripper to have fingers move in vertical direction
    left_arm.move_depth(.6)         # Move arm forward .6 meters
    left_arm.move_vertical(.4)      # Move arm up .4 meters
    

    right_arm.move_joint(2,  3*PI/4)  # Move Arm To vertical position
    right_arm.move_joint(4, -PI/2)    # Point gripper inwards
    right_arm.move_joint(5, -PI/2)    # Rotate gripper to have fingers move in vertical direction
    right_arm.move_depth(.6)
    right_arm.move_vertical(.4)
"""