#!/usr/bin/env python3

from robot import Robot
from protocol import RobotMessage
from factory import ComponentId

robot = Robot()
robot.start()
robot.send_message(RobotMessage(componentId=ComponentId.BASE, function="COUNTERCLOCKWISE", data={}))
robot.send_message(RobotMessage(componentId=ComponentId.LEFT_GRIPPER, function="close", data={}))
robot.send_message(RobotMessage(componentId=ComponentId.RIGHT_GRIPPER, function="close", data={}))
robot.send_message(RobotMessage(componentId=ComponentId.LEFT_GRIPPER, function="open", data={}))
robot.send_message(RobotMessage(componentId=ComponentId.RIGHT_GRIPPER, function="open", data={}))
robot.kill()
