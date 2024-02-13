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
from dexterity import Dexterity

#############################
"""
    These are the topic names of the action 
    servers to publish to to move the grippers
"""
LEFT_GRIPPER_TOPIC = '/left_gripper/command_robotiq_action/goal'
RIGHT_GRIPPER_TOPIC = '/right_gripper/command_robotiq_action/goal'
#############################

#############################
"""
    Here we'll define some constants related
    to the movement of the grippers to help
    fill our messages later
"""
DEFAULT_FORCE = 5.0	  # This is a reasonable amount of force

SPEED_MIN = 0.01
SPEED_MAX = 0.1
DEFAULT_SPEED = SPEED_MIN

POS_MIN = 0.025
POS_MAX = 0.085
POS_RANGE = POS_MAX - POS_MIN
POS_MID = POS_RANGE / 2

EMPTY_ID = ''
DEFAULT_SEQ_NUM = 0
############################
def choose_gripper_topic(dexterity):
    INVALID_DEXTERITY_MSG = "Invalid Arm Dexterity was Passed"
    if dexterity == Dexterity.LEFT:
        return LEFT_GRIPPER_TOPIC
    elif dexterity == Dexterity.RIGHT:
        return RIGHT_GRIPPER_TOPIC
    else:
        exit(INVALID_DEXTERITY_MSG)
############################
"""
    The only data we need on instantiation of the Gripper node are the publishers linked to our topics from above
"""
class GripperNode():
    def __init__(self, dexterity):
        self.pub = rospy.Publisher(choose_gripper_topic(dexterity), CommandRobotiqGripperActionGoal, queue_size=10)
    def close_gripper(self):
        publish_grip_message(self.pub, POS_MIN)
    def open_gripper(self):
        publish_grip_message(self.pub, POS_MAX)
    def half_grab(self):
        publish_grip_message(self.pub, POS_MID)
############################
        
############################
""" 
    This is a series of functions related to the population of the CommandRobotiqGripperActionGoal()
    message that needs to be published to the action server for the grippers to move
"""
def create_grip_msg(position):
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
    msg.goal.position = position	    # Here is where we can specify a desired position for the grippers
    msg.goal.speed = DEFAULT_SPEED	
    msg.goal.force = DEFAULT_FORCE
############################
 
############################
"""
    Now we have the very simple ROS interface that allows us
    to send our desired states to the grippers with some endpoints
"""
def publish_grip_message(publisher, position):
    msg = create_grip_msg(position)
    print(msg, publisher)
    publisher.publish(msg)
############################
