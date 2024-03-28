#!/usr/bin/env python

import rospy
import sys
from args import convert_dext_str_to_enum
from dexterity import Dexterity
from ur5e_arm import UR5e_Arm
from typing import List
from generic_ros_server import start_ros_server
from services import ServiceNames, ServicePorts
from req_resp import GenericRequest

def generic_callback(request: GenericRequest, arm: UR5e_Arm):
    function_call = getattr(arm, request.function)
    function_call(**request.args)

class Context():
    @staticmethod
    def get_dexterity_from_launch_arg() -> Dexterity:
        dexterity_arg: str = sys.argv[1]
        return convert_dext_str_to_enum(dexterity_arg)
    @staticmethod
    def create_node_name_from_dexterity(dexterity: Dexterity) -> str:
        node_suffix         = 'arm'
        dexterity_name: str = dexterity.__str__()
        return dexterity_name+"_"+node_suffix

def start_server(node_name: str):
    rospy.init_node(node_name, anonymous=True)
    arm = UR5e_Arm(DEXTERITY)
    start_ros_server(name=ServiceNames.LEFT_ARM, port=ServicePorts[ServiceNames.LEFT_ARM], 
                     callback=lambda request: generic_callback(request, arm))

DEXT_LAUNCH_ARM  = '~dexterity'
if __name__ == "__main__" :
    DEXTERITY: Dexterity = Context.get_dexterity_from_launch_arg()
    NODE_NAME: str = Context.create_node_name_from_dexterity(DEXTERITY)
    start_server(NODE_NAME)