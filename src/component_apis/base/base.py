#!/usr/bin/env python3

import rospy
from robot_types import AngularVelocity, LinearVelocity
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from speed_gen import TIME_STEP

HUSKY_BASE_TOPIC = '/cmd_vel'
FRAME_ID = 'base_link'
DEFAULT_SEQ_NUM = 0

class BaseNode():
    def __init__(self):
        self.base_pub = rospy.Publisher(HUSKY_BASE_TOPIC, Twist, queue_size=10)
    
    def create_base_msg(self, lin_vel, ang_vel):
        msg = Twist()
        self.fill_lin_vel(msg, lin_vel)
        self.fill_ang_vel(msg, ang_vel)
        return msg

    def fill_header(self, msg):
        msg.target_pose.header = Header()
        msg.target_pose.header.seq = DEFAULT_SEQ_NUM
        msg.target_pose.header.stamp = rospy.Time(0)
        msg.target_pose.header.frame_id = FRAME_ID

    def fill_lin_vel(self, msg, lin_vel):
        vector = Vector3()
        vector.x = lin_vel.x
        vector.y = lin_vel.y
        vector.z = lin_vel.z
        msg.linear = vector

    def fill_ang_vel(self, msg, ang_vel):
        vector = Vector3()
        vector.x = ang_vel.x
        vector.y = ang_vel.y
        vector.z = ang_vel.z
        msg.angular = vector

    def publish_base_message(self, position, orientation):
        msg = self.create_base_msg(position, orientation)
        print(msg, self.publisher)
        self.publisher.publish(msg)

    """ 
        From Clearpath Documentation 
        Set the linear.x value to drive the 
        robot forwards or backwards, and the angular.z value to rotate left or right.
    """
    def CLOCKWISE(self):
        lin_vel = LinearVelocity(0, 0, 0)
        ang_vel = AngularVelocity(0, 0, 0.5)
        self.publish_base_message(lin_vel, ang_vel)

    def COUNTERCLOCKISE(self):
        lin_vel = LinearVelocity(0, 0, 0)
        ang_vel = AngularVelocity(0, 0, -0.5)
        self.publish_base_message(lin_vel, ang_vel)

    def MOVE(self):
        lin_vel = LinearVelocity(0.2, 0, 0)
        ang_vel = AngularVelocity(0, 0, 0)
        self.publish_base_message(lin_vel, ang_vel)

    def REVERSE(self):
        lin_vel = LinearVelocity(-0.2, 0, 0)
        ang_vel = AngularVelocity(0, 0, 0)
        self.publish_base_message(lin_vel, ang_vel)

    def STOP(self):
        lin_vel = LinearVelocity(0, 0, 0)
        ang_vel = AngularVelocity(0, 0, 0)
        self.publish_base_message(lin_vel, ang_vel)

    def GO_FORWARD_DISTANCE(self, distance):
        VELOCITY = .2
        rate = rospy.Rate(1/TIME_STEP)
        point_velocities = int(distance / VELOCITY / TIME_STEP)
        for _ in range(1, point_velocities):
            lin_vel = LinearVelocity(VELOCITY, 0, 0)
            ang_vel = AngularVelocity(0, 0, 0)
            self.publish_base_message(lin_vel, ang_vel)
            rate.sleep()
