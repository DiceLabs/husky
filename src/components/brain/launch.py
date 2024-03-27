#!/usr/bin/python3

from robot import Sensors
from config import CAMERA_IDS
# from arm_client import call_arm
import rospy

# def init_robot():
#     robot = Robot()
#     robot.start()
#     return robot

def init_sensors():
    l_cam_serial = CAMERA_IDS.LEFT
    r_cam_serial = CAMERA_IDS.RIGHT
    sensors = Sensors(l_cam_serial, r_cam_serial)
    return sensors

# def init_brain(sensors: Sensors, robot: Robot):
#     brain = Brain(sensors, robot, tasks())
#     brain.run()
#     return brain

def init_hardware():
    sensors = init_sensors()
    # robot   = init_robot()
    # brain   = init_brain(sensors, robot)

def callback(event):
    print("hey")

NODE_NAME = 'main'
FREQUENCY = 20
PERIOD = 1/FREQUENCY
if __name__ == "__main__":
    init_hardware()
    rospy.init_node(NODE_NAME, anonymous=True)
    timer = rospy.timer(PERIOD, callback)

