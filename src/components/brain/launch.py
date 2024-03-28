#!/usr/bin/python3

from robot import Sensors
from config import CAMERA_IDS
from arm_client import call_arm
import rospy

def init_robot():
    robot = Robot()
    robot.start()
    return robot

def init_sensors():
    l_cam_serial = CAMERA_IDS.LEFT
    r_cam_serial = CAMERA_IDS.RIGHT
    sensors = Sensors(l_cam_serial, r_cam_serial)
    return sensors

def init_brain(sensors: Sensors, robot: Robot):
    brain = Brain(sensors, robot, tasks())
    brain.run()
    return brain

def init_hardware():
    sensors = init_sensors()
    robot   = init_robot()
    brain   = init_brain(sensors, robot)

def callback(event):
    print("hey")

""" def callback(sensors: Sensors):
    val = sensors.camera_r.camera_loop()
    print(val)
    if not val == None and val != (0.0, -0.0, 0.0):
        depth, dx, dy = val
        call_arm(dexterity=Dexterity.RIGHT, function="change_pose_ratchet", args={
            "x": f"{depth-.2}",
            "y": f"{dx}",
            "z": f"{dy}", 
            "blocking": "True"})
        
        # call_arm(dexterity=Dexterity.RIGHT, function="move_depth", args={"amount": f"{depth-.2}", "blocking": "True"})
        # call_arm(dexterity=Dexterity.RIGHT, function="move_horizontal", args={"amount": f"{dx}", "blocking": "True"})
        # call_arm(dexterity=Dexterity.RIGHT, function="move_vertical", args={"amount": f"{-dy}", "blocking": "True"})
        exit()
    
NODE_NAME = 'main'
FREQUENCY = 20
PERIOD = 1/FREQUENCY
if __name__ == "__main__":
    sensors = init_hardware()
    rospy.init_node(NODE_NAME, anonymous=True)
    # call_arm(dexterity=Dexterity.RIGHT, function="move_depth", args={"amount": "-.3", "blocking": "True"})
    timer = Timer(FREQUENCY, lambda: callback(sensors))
    timer.start()
 """

NODE_NAME = 'launch'
FREQUENCY = 20
PERIOD = 1/FREQUENCY
if __name__ == "__main__":
    init_hardware()
    rospy.init_node(NODE_NAME, anonymous=True)
    timer = rospy.Timer(rospy.Duration(PERIOD), callback)
    rospy.spin()

