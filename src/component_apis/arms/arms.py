#!/usr/bin/python3

""" 
    @author Conner Sommerfield
    @brief UR5e Arms API using ROS and Moveit.  
"""

import rospy
import moveit_commander
from dexterity import Dexterity
from geometry_msgs.msg import Pose, Quaternion
from robot_types import Position, Euler, PoseM, Quaternion
from transformations import quaternion_multiply, quaternion_from_euler
from conversions import degrees_to_radians

NODE_NAME = 'moveit_arm_api'
END_EFFECTOR_SUFFIX = "_ur_arm_wrist_3_link"
MANIPULATOR_PREFIX = "manipulator_"
INCREMENTAL_DISTANCE = 0.1 # units in meters
VELOCITY_SCALING_CONSTANT = 0.5 # reasonable speed for arms

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
        self.last_command = PoseM()
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
    def change_pose_abs(self, orientation: Quaternion, position: Position):
        """ 
            Will Move End Affector by relative amount passed in
            Uses 3-axis Position and Euler orientation with using units meters/radians
        """
        pose_goal = Pose()
        pose_goal.orientation.w = orientation.w
        pose_goal.orientation.x = orientation.x
        pose_goal.orientation.y = orientation.y
        pose_goal.orientation.z = orientation.z
        pose_goal.position.x = position.x
        pose_goal.position.y = position.y
        pose_goal.position.z = position.z
        self.group.set_pose_target(pose_goal, str(self.dexterity) + END_EFFECTOR_SUFFIX)
        self.group.set_max_velocity_scaling_factor(VELOCITY_SCALING_CONSTANT)
        self.group.go(wait=False)
        self.group.stop()
    def change_pose(self, orientation: Euler, position: Position):
        """ 
            Will Move End Affector by relative amount passed in
            Uses 3-axis Position and Euler orientation with using units meters/radians
        """
        yaw, pitch, roll = degrees_to_radians(yaw=orientation.yaw, pitch=orientation.pitch, roll=orientation.roll)
        pose_goal = self.create_pose_goal(Euler(yaw=yaw, pitch=pitch, roll=roll), position) 
        self.group.set_pose_target(pose_goal, str(self.dexterity) + END_EFFECTOR_SUFFIX)
        self.group.set_max_velocity_scaling_factor(VELOCITY_SCALING_CONSTANT)
        self.group.go(wait=False)
        self.group.stop()
        self.last_command = PoseM(orientation=orientation, position=position)
    def create_pose_goal(self, orientation: Euler, position: Position):
        pose_goal = Pose()
        current = self.group.get_current_pose().pose
        delta_orientation_quat = quaternion_from_euler(orientation.yaw, orientation.pitch, orientation.roll)
        current_orientation_quat = [current.orientation.w, current.orientation.x, current.orientation.y, current.orientation.z]
        goal_orientation_quat = quaternion_multiply(current_orientation_quat, delta_orientation_quat)
        pose_goal.orientation.w = goal_orientation_quat[0]
        pose_goal.orientation.x = goal_orientation_quat[1]
        pose_goal.orientation.y = goal_orientation_quat[2]
        pose_goal.orientation.z = goal_orientation_quat[3]
        pose_goal.position.x = current.position.x + position.x
        pose_goal.position.y = current.position.y + position.y
        pose_goal.position.z = current.position.z + position.z
        return pose_goal

    def move_vertical(self, amount: float):
        self.change_pose(Euler(), Position(z=amount))
    def move_horizontal(self, amount: float):
        self.change_pose(Euler(), Position(y=amount))
    def move_depth(self, amount: float):
        self.change_pose(Euler(), Position(x=amount))

    def yaw(self, amount):
        self.change_pose(Euler(yaw=amount), Position())
    def pitch(self, amount):
        self.change_pose(Euler(pitch=amount), Position())
    def roll(self, amount):
        self.change_pose(Euler(roll=amount), Position())
        
    def move_up(self):
        self.move_vertical(INCREMENTAL_DISTANCE)
    def move_down(self):
        self.move_vertical(-INCREMENTAL_DISTANCE)
    def move_left(self):
        self.move_horizontal(INCREMENTAL_DISTANCE)
    def move_right(self):
        self.move_horizontal(-INCREMENTAL_DISTANCE)
    def move_forward(self):
        self.move_depth(INCREMENTAL_DISTANCE)
    def move_backward(self):
        self.move_depth(-INCREMENTAL_DISTANCE)
        
    def undo_last_command(self):
        undo_command = self.last_command
        for key in undo_command.position.__dict__:
            setattr(self, key, getattr(undo_command.position, key) * -1)
        for key in undo_command.orientation.__dict__:
            setattr(self, key, getattr(undo_command.orientation, key) * -1)
        self.change_pose(undo_command.orientation, undo_command.position)

if __name__ == "__main__" :
    rospy.init_node(NODE_NAME, anonymous=True)
    left_arm = UR5e_Arm(Dexterity.LEFT)
    right_arm = UR5e_Arm(Dexterity.RIGHT)
    left_arm.print_info()
    right_arm.print_info()