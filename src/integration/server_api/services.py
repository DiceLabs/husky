#!/usr/bin/env python3

class ServiceNames():
    BASE = "BASE"
    LEFT_ARM = "LEFT_ARM"
    RIGHT_ARM = "RIGHT_ARM"
    LEFT_CAMERA = "LEFT_CAMERA"
    RIGHT_CAMERA = "RIGHT_CAMERA"
    LEFT_GRIPPER = "LEFT_GRIPPER"
    RIGHT_GRIPPER = "RIGHT_GRIPPER" 

ServicePorts = {
    ServiceNames.LEFT_CAMERA:   6000,
    ServiceNames.RIGHT_CAMERA:  6001,
    ServiceNames.LEFT_GRIPPER:  6002,
    ServiceNames.RIGHT_GRIPPER: 6003,
    ServiceNames.LEFT_ARM:      6004,
    ServiceNames.RIGHT_ARM:     6005,
    ServiceNames.BASE:          6006
}