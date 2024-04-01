#!/usr/bin/python3

""" 
    @author Conner Sommerfield
    @brief UR5e Arms API using ROS and Moveit.  
"""

import os
import moveit_commander
from dexterity import Dexterity
from geometry_msgs.msg import Pose, Quaternion
from robot_types import Position, Euler, PoseM, Quaternion
from transformations import quaternion_multiply, quaternion_from_euler
from conversions import degrees_to_radians

NAMESPACE_VAR = "ROS_NAMESPACE"
NAMESPACE_SUFFIX = "_ur"
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
        print ("============ End effector: %s" % self.group.get_end_effector_link())
        print ("============ Current State: %s" % self.group.get_current_state())
        print ("============ Robot Groups:", self.robot.get_group_names())
        print ("============ Robot Pose:", self.group.get_current_pose().pose)

    def move_joint(self, joint_id: int, amount: float, blocking: bool=True):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[joint_id] += amount
        self.group.go(joint_goal, wait=blocking)
        self.group.stop()

    def make_moveit_pose_request(self, pose_goal: Pose, blocking: bool=True):
        end_effector_name = str(self.dexterity) + END_EFFECTOR_SUFFIX
        self.group.set_pose_target(pose_goal, end_effector_name)
        self.group.go(wait=blocking)
        self.group.stop()

    def construct_abs_pose_goal(self, orientation: Quaternion, position: Position,):
        """ 
            Basically a ROS type conversion
        """
        pose_goal = Pose()
        pose_goal.orientation.w = orientation.w
        pose_goal.orientation.x = orientation.x
        pose_goal.orientation.y = orientation.y
        pose_goal.orientation.z = orientation.z
        pose_goal.position.x = position.x
        pose_goal.position.y = position.y
        pose_goal.position.z = position.z
        return pose_goal
    def change_pose_abs(self, orientation: Quaternion, position: Position, blocking: bool=True):
        """ 
            Will Move End Affector by relative amount passed in
            Uses 3-axis Position and Euler orientation with using units meters/radians
        """
        pose_goal = self.construct_abs_pose_goal(orientation=orientation, position=position)
        self.make_moveit_pose_request(blocking=blocking, pose_goal=pose_goal)

    def construct_abs_pos_goal(self, position: Position):
        """ 
            Basically a ROS type conversion
        """
        current_pose = self.group.get_current_pose().pose
        pose_goal = Pose()
        pose_goal.orientation.w = current_pose.orientation.w
        pose_goal.orientation.x = current_pose.orientation.x
        pose_goal.orientation.y = current_pose.orientation.y
        pose_goal.orientation.z = current_pose.orientation.z
        pose_goal.position.x = position.x
        pose_goal.position.y = position.y
        pose_goal.position.z = position.z
        return pose_goal
    def change_pos_abs(self, position: Position, blocking: bool=True):
        """ 
            Will Move End Affector by relative amount passed in
            Uses 3-axis Position and Euler orientation with using units meters/radians
        """
        pose_goal = self.construct_abs_pos_goal(position=position)
        self.make_moveit_pose_request(blocking=blocking, pose_goal=pose_goal)

    def construct_rel_pose_goal(self, orientation: Euler, position: Position):
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
    def change_pose_rel(self, orientation: Euler, position: Position, blocking: bool=True):
        """ 
            Will Move End Affector by relative amount passed in
            Uses 3-axis Position and Euler orientation with using units meters/radians
        """
        yaw, pitch, roll = degrees_to_radians(yaw=orientation.yaw, pitch=orientation.pitch, roll=orientation.roll)
        pose_goal = self.construct_rel_pose_goal(Euler(yaw=yaw, pitch=pitch, roll=roll), position) 
        self.make_moveit_pose_request(blocking=blocking, pose_goal=pose_goal)
        self.last_command = PoseM(orientation=orientation, position=position)
    
    # def change_pose_ros(self, x, y, z, yaw, pitch, roll, blocking: bool):
    #     """ 
    #         Will Move End Affector by relative amount passed in
    #         Uses 3-axis Position and Euler orientation with using units meters/radians
    #     """
    #     pose_goal = self.construct_rel_pose_goal(Euler(yaw=0, pitch=0, roll=0), Position(x=x, y=y, z=z)) 
    #     self.make_moveit_pose_request(blocking=blocking, pose_goal=pose_goal)
    #     self.last_command = PoseM(Euler(yaw=0, pitch=0, roll=0), Position(x=x, y=y, z=z))

    def move_vertical(self, amount: float, blocking: bool=True):
        self.change_pose_rel(Euler(), Position(z=amount), blocking)
    def move_horizontal(self, amount: float, blocking: bool=True):
        self.change_pose_rel(Euler(), Position(y=amount), blocking)
    def move_depth(self, amount: float, blocking: bool=True):
        self.change_pose_rel(Euler(), Position(x=amount), blocking)

    def yaw(self, amount, blocking: bool=True):
        self.change_pose_rel(Euler(yaw=amount), Position(), blocking)
    def pitch(self, amount, blocking: bool=True):
        self.change_pose_rel(Euler(pitch=amount), Position(), blocking)
    def roll(self, amount, blocking: bool=True):
        self.change_pose_rel(Euler(roll=amount), Position(), blocking)
        
    def move_up(self, blocking: bool=True):
        self.move_vertical(INCREMENTAL_DISTANCE, blocking)
    def move_down(self, blocking: bool=True):
        self.move_vertical(-INCREMENTAL_DISTANCE, blocking)
    def move_left(self, blocking: bool=True):
        self.move_horizontal(INCREMENTAL_DISTANCE, blocking)
    def move_right(self, blocking: bool=True):
        self.move_horizontal(-INCREMENTAL_DISTANCE, blocking)
    def move_forward(self, blocking: bool=True):
        self.move_depth(INCREMENTAL_DISTANCE, blocking)
    def move_backward(self, blocking: bool=True):
        self.move_depth(-INCREMENTAL_DISTANCE, blocking)
            
    def undo_last_command(self):
        undo_command = self.last_command
        for key in undo_command.position.__dict__:
            setattr(self, key, getattr(undo_command.position, key) * -1)
        for key in undo_command.orientation.__dict__:
            setattr(self, key, getattr(undo_command.orientation, key) * -1)
        self.change_pose_rel(undo_command.orientation, undo_command.position, True)

