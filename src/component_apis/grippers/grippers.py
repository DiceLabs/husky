#!/usr/bin/env python

""" 
    @Zix
    Gripper API
    Defines Easy ROS Interface to call common movements for grippers (open, close ...)
    Must be run from environment with correct ROS msgs set up, the husky will have this environment on its on-board computer
"""

import rospy
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperActionGoal
from std_msgs.msg import Header
from pynput import keyboard
from pynput.keyboard import Key

LEFT_GRIPPER_TOPIC = '/left_gripper/command_robotiq_action/goal'
RIGHT_GRIPPER_TOPIC = '/right_gripper/command_robotiq_action/goal'

""" Define movement constants """
DEFAULT_FORCE = 5.0	# Reasonable amount of force

SPEED_MIN = 0.01
SPEED_MAX = 0.1
DEFAULT_SPEED = SPEED_MIN

POS_MIN = 0.00
POS_MAX = 0.085
POS_RANGE = POS_MAX - POS_MIN
POS_MID = POS_RANGE / 2
""" Define movement constants """

EMPTY_ID = ''
DEFAULT_SEQ_NUM = 0

""" namespace MessageFilling() """
def create_msg(position):
    msg = CommandRobotiqGripperActionGoal()
    fill_header(msg)
    fill_goal_id(msg)
    fill_goal(msg, position)
    return msg

def fill_header(msg):
    msg.header = Header()
    msg.header.seq = DEFAULT_SEQ_NUM
    msg.header.stamp = rospy.Time(0)
    msg.header.frame_id = EMPTY_ID

def fill_goal_id(msg):
    msg.goal_id = CommandRobotiqGripperActionGoal().goal_id
    msg.goal_id.stamp = rospy.Time(0)
    msg.goal_id.id = EMPTY_ID

def fill_goal(msg, position):
    msg.goal.emergency_release = False
    msg.goal.emergency_release_dir = 0
    msg.goal.stop = False
    msg.goal.position = position	
    msg.goal.speed = DEFAULT_SPEED	
    msg.goal.force = DEFAULT_FORCE
""" namespace MessageFilling() """

class GripperNode():
    def __init__(self):
        self.left_pub = rospy.Publisher(LEFT_GRIPPER_TOPIC, CommandRobotiqGripperActionGoal, queue_size=10)
        self.right_pub = rospy.Publisher(RIGHT_GRIPPER_TOPIC, CommandRobotiqGripperActionGoal, queue_size=10)

def close_gripper(publisher):
    msg = create_msg(POS_MIN)
    print(msg, publisher)
    publisher.publish(msg)

def open_gripper(publisher):
    msg = create_msg(POS_MAX)
    print(msg, publisher)
    publisher.publish(msg)


