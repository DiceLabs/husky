#!/usr/bin/python3

""" 
    @author Conner Sommerfield
    @brief Test UR5e Arms API by Creating Mock And Ignoring ROS Dependency
"""

from dexterity import Dexterity
from geometry_msgs.msg import Pose
from robot_types import Position, Euler, PoseM
from transformations import quaternion_multiply, quaternion_from_euler, euler_from_quaternion
from conversions import degrees_to_radians
import copy

NODE_NAME = 'moveit_arm_api'
END_EFFECTOR_SUFFIX = "_ur_arm_wrist_3_link"
MANIPULATOR_PREFIX = "manipulator_"
INCREMENTAL_DISTANCE = 0.1 # units in meters
VELOCITY_SCALING_CONSTANT = 0.5 # reasonable speed for arms

class Mock_UR5e_Arm:
    """ 
        Arm can be initialized as left or right. Connects to moveit client and can take advantage of various functions of moveit API,
        notably the ability to set joint targets or a pose target, other functions have been included to ease the use of common movements 
    """
    def __init__(self, dexterity: Dexterity):
        self.dexterity = dexterity
    def move_joint(self, joint_id: int, amount: float):
        joint_goal = [0,0,0,0,0,0]
        joint_goal[joint_id] += amount
        return joint_goal
    def change_pose(self, orientation: Euler, position: Position):
        """ 
            Will Move End Affector by relative amount passed in
            Uses 3-axis Position and Euler orientation with using units meters/radians
        """
        yaw, pitch, roll = degrees_to_radians(yaw=orientation.yaw, pitch=orientation.pitch, roll=orientation.roll)
        pose_goal = self.create_pose_goal(Euler(yaw=yaw, pitch=pitch, roll=roll), position) 
        return pose_goal
    def create_pose_goal(self, orientation: Euler, position: Position):
        pose_goal = Pose()
        current = Pose()
        current.position.x = 10
        current.position.y = 10
        current.position.z = 20
        yaw, pitch, roll = degrees_to_radians(10,20,30)
        quaternion_orientation = quaternion_from_euler(yaw, pitch, roll)
        current.orientation.w = quaternion_orientation[0]
        current.orientation.x = quaternion_orientation[1]
        current.orientation.y = quaternion_orientation[2]
        current.orientation.z = quaternion_orientation[3]

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
        return self.change_pose(Euler(), Position(z=amount))
    def move_horizontal(self, amount: float):
        return self.change_pose(Euler(), Position(y=amount))
    def move_depth(self, amount: float):
        return self.change_pose(Euler(), Position(x=amount))

    def yaw(self, amount):
        return self.change_pose(Euler(yaw=amount), Position())
    def pitch(self, amount):
        return self.change_pose(Euler(pitch=amount), Position())
    def roll(self, amount):
        return self.change_pose(Euler(roll=amount), Position())
        
    def move_up(self):
        return self.move_vertical(INCREMENTAL_DISTANCE)
    def move_down(self):
        return self.move_vertical(-INCREMENTAL_DISTANCE)
    def move_left(self):
        return self.move_horizontal(INCREMENTAL_DISTANCE)
    def move_right(self):
        return self.move_horizontal(-INCREMENTAL_DISTANCE)
    def move_forward(self):
        return self.move_depth(INCREMENTAL_DISTANCE)
    def move_backward(self):
        return self.move_depth(-INCREMENTAL_DISTANCE)

def test_pose_positions():
    mock_arm = Mock_UR5e_Arm(Dexterity.LEFT)
    expected_goal = Pose()
    expected_goal.position.x = 10
    expected_goal.position.y = 20
    expected_goal.position.z = 30
    goal = mock_arm.change_pose(Euler(), Position(y=10, z=10))
    goal.orientation.w = 0
    goal.orientation.x = 0
    goal.orientation.y = 0
    goal.orientation.z = 0
    try:    
        assert goal == expected_goal
    except:
        print(f"Expected\n {str(expected_goal)}\n But Got\n {str(goal)}")
        return
    print("test position passed!")

def test_undo_last_command():
    expected_euler = PoseM(orientation=Euler(yaw=20, pitch=20, roll=20), position=Position(x=10, y=10, z=10))
    actual_euler = copy.deepcopy(expected_euler)
    for key in actual_euler.position.__dict__.keys():
        setattr(actual_euler.position, key, getattr(actual_euler.position, key) * -1)
        
    for key in actual_euler.orientation.__dict__.keys():
        setattr(actual_euler.orientation, key, getattr(actual_euler.orientation, key) * -1)
    try:
        assert actual_euler.orientation.yaw == -20
        assert actual_euler.orientation.pitch == -20
        assert actual_euler.orientation.roll == -20
        assert actual_euler.position.x == -10
        assert actual_euler.position.y == -10
        assert actual_euler.position.z == -10
    except AssertionError:
        print(f"Test Failed!\nExpected\n{expected_euler}\nBut Got\n{actual_euler}")
        return
    print("Test undo passed!")

     
if __name__ == "__main__":
    test_pose_positions()
    test_undo_last_command()
