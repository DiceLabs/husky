#!/usr/bin/python3

from robot import Robot, Sensors
from brain import Brain
from config import tasks, CAMERA_IDS

def init_robot():
    robot = Robot()
    robot.start()
    return robot

def init_sensors():
    l_cam_serial = CAMERA_IDS.LEFT
    r_cam_serial = CAMERA_IDS.RIGHT
    sensors = Sensors(l_cam_serial, r_cam_serial)
    return sensors

def init_brain():
    brain = Brain(sensors, robot, tasks())
    brain.run()
    return brain

if __name__ == "__main__":
    sensors = init_sensors()
    robot   = init_robot()
    brain   = init_brain()
