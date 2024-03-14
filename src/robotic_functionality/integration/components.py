#!/usr/bin/env python3

from enum import Enum
from grippers import GripperNode
from base import BaseNode
from arms import UR5e_Arm
from dexterity import Dexterity

class ComponentId(Enum):
    BASE            = 0
    LEFT_ARM        = 1
    RIGHT_ARM       = 2
    LEFT_GRIPPER    = 3
    RIGHT_GRIPPER   = 4

def ComponentFactory():
    return  {
                ComponentId.BASE          : BaseNode(),
                ComponentId.LEFT_ARM      : UR5e_Arm(dexterity=Dexterity.LEFT),
                ComponentId.RIGHT_ARM     : UR5e_Arm(dexterity=Dexterity.RIGHT),
                ComponentId.LEFT_GRIPPER  : GripperNode(dexterity=Dexterity.LEFT),
                ComponentId.RIGHT_GRIPPER : GripperNode(dexterity=Dexterity.RIGHT),
            }
