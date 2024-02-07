#!/usr/bin/env python3

import rospy
from robot_types import AngularVelocity, LinearVelocity
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from speed_gen import generate_speed_profile, TIME_STEP

HUSKY_BASE_TOPIC = '/cmd_vel'
FRAME_ID = 'base_link'
DEFAULT_SEQ_NUM = 0

class BaseNode():
    def __init__(self):
        self.base_pub = rospy.Publisher(HUSKY_BASE_TOPIC, Twist, queue_size=10)

def create_base_msg(lin_vel, ang_vel):
    msg = Twist()
    fill_lin_vel(msg, lin_vel)
    fill_ang_vel(msg, ang_vel)
    return msg

def fill_header(msg):
    msg.target_pose.header = Header()
    msg.target_pose.header.seq = DEFAULT_SEQ_NUM
    msg.target_pose.header.stamp = rospy.Time(0)
    msg.target_pose.header.frame_id = FRAME_ID

def fill_lin_vel(msg, lin_vel):
    vector = Vector3()
    vector.x = lin_vel.x
    vector.y = lin_vel.y
    vector.z = lin_vel.z
    msg.linear = vector

def fill_ang_vel(msg, ang_vel):
    vector = Vector3()
    vector.x = ang_vel.x
    vector.y = ang_vel.y
    vector.z = ang_vel.z
    msg.angular = vector

def publish_base_message(publisher, position, orientation):
    msg = create_base_msg(position, orientation)
    print(msg, publisher)
    publisher.publish(msg)

""" 
    From Clearpath Documentation 
    Set the linear.x value to drive the 
    robot forwards or backwards, and the angular.z value to rotate left or right.
"""

def CLOCKWISE(publisher):
    lin_vel = LinearVelocity(0, 0, 0)
    ang_vel = AngularVelocity(0, 0, 0.5)
    publish_base_message(publisher, lin_vel, ang_vel)

def COUNTERCLOCKISE(publisher):
    lin_vel = LinearVelocity(0, 0, 0)
    ang_vel = AngularVelocity(0, 0, -0.5)
    publish_base_message(publisher, lin_vel, ang_vel)

def MOVE(publisher):
    lin_vel = LinearVelocity(0.2, 0, 0)
    ang_vel = AngularVelocity(0, 0, 0)
    publish_base_message(publisher, lin_vel, ang_vel)

def REVERSE(publisher):
    lin_vel = LinearVelocity(-0.2, 0, 0)
    ang_vel = AngularVelocity(0, 0, 0)
    publish_base_message(publisher, lin_vel, ang_vel)

def STOP(publisher):
    lin_vel = LinearVelocity(0, 0, 0)
    ang_vel = AngularVelocity(0, 0, 0)
    publish_base_message(publisher, lin_vel, ang_vel)

def GO_FORWARD_DISTANCE(publisher, distance):
    VELOCITY = .2
    rate = rospy.Rate(1/TIME_STEP)
    point_velocities = int(distance / VELOCITY / TIME_STEP)
    for _ in range(1, point_velocities):
        lin_vel = LinearVelocity(VELOCITY, 0, 0)
        ang_vel = AngularVelocity(0, 0, 0)
        publish_base_message(publisher, lin_vel, ang_vel)
        rate.sleep()