#!/usr/bin/env python3

from enum import Enum
from grippers import GripperNode
from base import BaseNode
from arms import UR5e_Arm
from dexterity import Dexterity

class GenericComponentInit():
    def __init__(self, name, component, args):
        self.component = component
        self.args = args
        self.name = name

class ComponentId(Enum):
    BASE            = 0
    LEFT_ARM        = 1
    RIGHT_ARM       = 2
    LEFT_GRIPPER    = 3
    RIGHT_GRIPPER   = 4

def ComponentFactory():
    return  {
                ComponentId.BASE          : GenericComponentInit("Base", BaseNode, {}),
                # ComponentId.LEFT_ARM      : GenericComponentInit(UR5e_Arm, {"dexterity":Dexterity.LEFT}),
                # ComponentId.RIGHT_ARM     : GenericComponentInit(UR5e_Arm, {"dexterity":Dexterity.RIGHT}),
                ComponentId.LEFT_GRIPPER  : GenericComponentInit("Left_Gripper", GripperNode, {"dexterity":Dexterity.LEFT}),
                ComponentId.RIGHT_GRIPPER : GenericComponentInit("Right_Gripper", GripperNode, {"dexterity":Dexterity.RIGHT}),
            }
