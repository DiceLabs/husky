#!/usr/bin/env python

import rospy
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperActionGoal
from std_msgs.msg import Header

LEFT_GRIPPER_TOPIC = '/left_gripper/command_robotiq_action/goal'
RIGHT_GRIPPER_TOPIC = '/right_gripper/command_robotiq_action/goal'

DEFAULT_FORCE = 5.0	# Reasonable amount of force

# speed calue limits from 0.01 to 0.1
SPEED_MIN = 0.01
SPEED_MAX = 0.1
DEFAULT_SPEED = SPEED_MIN

# position value limits from 0.00 to 0.085
POS_MIN = 0.00
POS_MAX = 0.085
POS_RANGE = POS_MAX - POS_MIN
POS_MID = POS_RANGE / 2

EMPTY_ID = ''
DEFAULT_SEQ_NUM = 0

class GripperNode():
    def __init__(self):
        self.left_pub = rospy.Publisher(LEFT_GRIPPER_TOPIC, CommandRobotiqGripperActionGoal, queue_size=10)
        self.right_pub = rospy.Publisher(RIGHT_GRIPPER_TOPIC, CommandRobotiqGripperActionGoal, queue_size=10)
    
class MessageFiller():
    def create_msg(self, position):
        msg = CommandRobotiqGripperActionGoal()
        self.fill_header(msg)
        self.fill_goal_id(msg)
        self.fill_goal(msg, position)
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

def close_gripper(message_filler, publisher):
    msg = message_filler.create_msg(POS_MIN)
    publisher.pub(msg)

def open_gripper(message_filler, publisher):
    msg = message_filler.create_msg(POS_MAX)
    publisher.pub(msg)

def init_node():
    rospy.init_node('gripper_node')
    rospy.loginfo("GRIPPER NODE TURNED ON")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("########################################################################################")
    rospy.loginfo("Welcome to the gripper node! This node will give you control of the robot grippers")
    rospy.loginfo("You can use the keyboard input to give common commands to the grippers")
    rospy.loginfo("1. Open Left Gripper")
    rospy.loginfo("2. Close Left Gripper")
    rospy.loginfo("3. Open Right Gripper")
    rospy.loginfo("4. Close Right Gripper")
    rospy.loginfo("########################################################################################")

def spin_ros():
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__" :
    spin_ros()
