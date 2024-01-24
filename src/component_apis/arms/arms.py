#!/usr/bin/python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

class robot_arm:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator_left"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.current_pose = self.move_group.get_current_pose().pose
        self.current_quaternion = [self.current_pose.orientation.x, self.current_pose.orientation.y,
                                   self.current_pose.orientation.z, self.current_pose.orientation.w]
        self.current_euler = euler_from_quaternion(self.current_quaternion)
        self.current_position = [self.current_pose.position.x, self.current_pose.position.y,
                                 self.current_pose.position.z]
        self.target_pose = Pose()
        self.object_center_x = None
        self.object_center_y = None
        self.delta_x = None
        self.delta_x_old = None
        self.delta_y = None
        self.delta_y_old = None
        self.delta_z = None
        self.delta_z_old = None

    def set_initial_arm_params(self):
        self.target_pose.position.x = 0.3553980405176474
        self.target_pose.position.y = 0.2636410314410824
        self.target_pose.position.z = 0.75
        self.target_pose.orientation.x = 0.5
        self.target_pose.orientation.y = 0.5
        self.target_pose.orientation.z = 0.5
        self.target_pose.orientation.w = 0.5
        self.move_group.set_pose_target(self.target_pose)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        plan = self.move_group.go(wait=True)
        if plan:
            rospy.loginfo("Initial Parameters: =====SUCCESSFUL=====")
        else:
            rospy.logerr("Initial Parameters: =====FAILED=====")

    def get_current_pose(self):
        self.target_pose = self.current_pose 
        CURRENT_POSE_MESSAGE = f"Current Pose is: {self.target_pose}"
        print(CURRENT_POSE_MESSAGE)  

    def move_up(self):
        self.target_pose.position.z = (self.current_pose.position.z + 0.02)
        print(self.target_pose.position.z)

    def move_down(self):
        self.target_pose.position.z = (self.current_pose.position.z - 0.02)
        print(self.target_pose.position.z)

    def move_left(self):
        self.target_pose.position.y = (self.current_pose.position.y - 0.02)
        print(self.target_pose.position.y)

    def move_right(self):
        self.target_pose.position.y = (self.current_pose.position.y + 0.02)
        print(self.target_pose.position.y)

    def move_forward(self):
        self.target_pose.position.x = (self.current_pose.position.x + 0.02)
        print(self.target_pose.position.x)

    def move_backward(self):
        self.target_pose.position.x = (self.current_pose.position.x - 0.02)
        print(self.target_pose.position.x)


