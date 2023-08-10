#!/usr/bin/python3
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from std_msgs.msg import Header
import moveit_commander
import geometry_msgs.msg
import sys
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi

from std_msgs.msg import Float64MultiArray

class robot_arm:
    def __init__(self):
        self.Center_x = None
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
        self.target_pose = geometry_msgs.msg.Pose()
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


    def set_old_deltas(self):
        self.delta_x_old = self.delta_x
        self.delta_y_old = self.delta_y
        self.delta_z_old = self.delta_z

    def set_orientation(self):
        self.target_pose.orientation.x = 0.5
        self.target_pose.orientation.y = 0.5
        self.target_pose.orientation.z = 0.5
        self.target_pose.orientation.w = 0.5

    def get_current_pose(self):
        self.target_pose = self.current_pose  #### self.move_group.get_current_pose().pose
        print(f"Current Pose is: {self.target_pose}")  ######## MAKE SURE ITS CHANGING

    def move_up(self):
        # self.current_pose = self.current_pose
        self.target_pose.position.z = (self.current_pose.position.z + 0.02)

    def move_down(self):
        # self.current_pose = self.current_pose
        self.target_pose.position.z = (self.current_pose.position.z - 0.02)

    def move_left(self):
        # self.current_pose = self.current_pose
        self.target_pose.position.y = (self.current_pose.position.y - 0.02)

    def move_right(self):
        # self.current_pose = self.current_pose
        self.target_pose.position.y = (self.current_pose.position.y + 0.02)

    def move_forward(self):
        # self.current_pose = self.current_pose
        self.target_pose.position.x = (self.current_pose.position.x + 0.02)

    def move_backward(self):
        # self.current_pose = self.current_pose
        self.target_pose.position.x = (self.current_pose.position.x - 0.02)

        # Establishing the conditions for moving the arm left or right based on bottle position

    def move_conditions_y(self):
        try:
            if self.delta_x < 0:
                print("Object is to the right >> moving right")
                self.move_right()

            elif self.delta_x > 0:
                print("Object is to the left >> moving left")
                self.move_left()
        except TypeError:
            print("Move Conditions X Type Error")

        # Establishing the conditions for moving the arm up or down based on bottle position

    def move_conditions_z(self):
        try:
            if self.delta_y < 0:
                print(f"Object is lower >> moving down")
                self.move_down()

            # elif
            if self.delta_y > 0:
                print(f"Object is higher >> moving up")
                self.move_up()
        except TypeError:
            print("Move Conditions X Type Error")

        # The method that pulls it all together

    def move_conditions_x(self):
        try:
            if self.delta_z < 0.4:
                print(f"Object is close >> moving backwards")
                self.move_backward()

            # elif
            if self.delta_z > 0.4:
                print(f"Object is far>> moving forward")
                self.move_forward()
        except TypeError:
            print("Move Conditions X Type Error")

    def move_arms(self,msg):
        self.delta_x,self.delta_y, self.delta_z = msg.data
        print("DELTAS: ", self.delta_x,", ", self.delta_y, ",", self.delta_z)
        # if self.delta_x != self.delta_x_old or self.delta_y != self.delta_y_old: #For Testing arm movement in y(left/right and z(up/down)
        if self.delta_y != self.delta_y_old or self.delta_x != self.delta_x_old or self.delta_z != self.delta_z_old :  ###Testing (up/down) movement only
            # if self.delta_x != self.delta_x_old: ###Testing (right/left) movement only
            self.set_old_deltas()
            self.get_current_pose()
            self.move_conditions_y() #setting new pose.positions (left/right)
            self.move_conditions_z()  # setting new pose.positions (up/down)
            self.move_conditions_x()  # setting new pose.positions (forward/backward)
            self.set_orientation()
            self.move_group.set_pose_target(self.target_pose)
            self.move_group.set_max_velocity_scaling_factor(0.1)
            plan = self.move_group.go(wait=True)
            if plan:
                rospy.loginfo("Motion planning and execution succeeded!")
            else:
                rospy.logerr("Motion planning and execution failed!")



def main():
    rospy.init_node("arm_follow")
    tester = robot_arm()
    # tester.camera_intrinsics = rospy.wait_for_message("/camera/depth/camera_info", CameraInfo)
    tester.set_initial_arm_params()
    print("=======================INITIAL PARAMS=============================")


    rospy.Subscriber("/deltas", Float64MultiArray, tester.move_arms, queue_size=1, buff_size=2_000_000)

    #while not rospy.is_shutdown():
    #    print("STAAARTED")
    #    tester.calc()
    #    if tester.bottle_found:
            #self.bottle_found = False
    #        tester.move_arms()


    # tester.detect()
    # tester.detect_depth()

    rospy.spin()


# except rospy.ROSInterruptException:
#    pass


if __name__ == "__main__":
    main()