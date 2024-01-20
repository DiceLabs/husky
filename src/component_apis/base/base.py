#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

HUSKY_BASE_TOPIC = '/move_base_simple/goal'
FRAME_ID = 'base_link'
DEFAULT_SEQ_NUM = 0

class BaseNode():
    def __init__(self):
        self.base_pub = rospy.Publisher(HUSKY_BASE_TOPIC, PoseStamped, queue_size=10)

class Position():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Orientation():
    def __init__(self, x, y, z, w):
        self.x = x 
        self.y = y
        self.z = z
        self.w = w

def create_base_msg(position, orientation):
    msg = PoseStamped()
    fill_header(msg)
    fill_position(msg, position)
    fill_orientation(msg, orientation)
    return msg

def fill_header(msg):
    msg.header = Header()
    msg.header.seq = DEFAULT_SEQ_NUM
    msg.header.stamp = rospy.Time(0)
    msg.header.frame_id = FRAME_ID

def fill_position(msg, position):
    point = Point()
    point.x = position.x
    point.y = position.y
    point.z = position.z
    msg.position = point

def fill_orientation(msg, orientation):
    quaternion = Quaternion()
    quaternion.x = orientation.x
    quaternion.y = orientation.y
    quaternion.z = orientation.z
    quaternion.w = orientation.w
    msg.orientation = quaternion

def publish_base_message(publisher, position, orientation):
    msg = create_base_msg(position, orientation)
    print(msg, publisher)
    publisher.publish(msg)

def TURN(publisher):
    position = Position(0, 0, 0)
    orientation = Orientation(0.1, 0, 0, 0)
    publish_base_message(publisher, position, orientation)

def FORWARD(publisher):
    position = Position(0.1, 0, 0)
    orientation = Orientation(0, 0, 0, 0)
    publish_base_message(publisher, position, orientation)

"""  scp -r dicelabs administrator@146.244.98.51:/home/administrator/catkin_ws/temp
"""