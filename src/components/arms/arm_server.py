#!/usr/bin/env python

import rospy
import sys
from args import convert_dext_str_to_enum
from dexterity import Dexterity
from ur5e_arm import UR5e_Arm
from robot_msgs.srv import GenericSrvRequest, GenericSrvResponse, GenericSrv
from robot_msgs.msg import KeyValue
from protocol import GenericComponent, RobotMessage
from factory import ComponentId, GenericComponentMetaData
from multiprocessing import Queue
from typing import List
from factory import convert_int_to_id

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
    @staticmethod
    def convert_dexterity_to_component(dexterity: Dexterity) -> ComponentId:
        component_map = {
            Dexterity.LEFT : ComponentId.LEFT_ARM,
            Dexterity.RIGHT: ComponentId.RIGHT_ARM
        }
        return component_map[dexterity]
    @staticmethod
    def get_arm_meta_data(node_name: str, dexterity: Dexterity) -> GenericComponentMetaData:
        DEXT_ARG_NAME       = "dexterity"
        args = {DEXT_ARG_NAME: dexterity}
        return GenericComponentMetaData(node_name, UR5e_Arm, args)


def arm_callback(request: GenericSrvRequest) -> GenericSrvResponse:
    args = {}
    entryList : List[KeyValue] = request.message.args
    for entry in entryList:
        args[entry.key] = entry.value
    component_id: ComponentId = convert_int_to_id(request.message.componentId)
    robot_msg = RobotMessage(component_id, request.message.function, args)
    GENERIC_ARM.handle_message(robot_msg)
    return GenericSrvResponse()


def start_server(node_name: str):
    rospy.init_node(node_name, anonymous=True)
    rospy.Service(node_name, GenericSrv, arm_callback)
    rospy.spin()


DEXT_LAUNCH_ARM  = '~dexterity'
if __name__ == "__main__" :
    DEXTERITY: Dexterity = Context.get_dexterity_from_launch_arg()
    NODE_NAME: str = Context.create_node_name_from_dexterity(DEXTERITY)
    ARM_META_DATA: GenericComponentMetaData = Context.get_arm_meta_data(NODE_NAME, DEXTERITY)
    COMPONENT_ID: ComponentId = Context.convert_dexterity_to_component(DEXTERITY)
    GENERIC_ARM = GenericComponent(COMPONENT_ID, ARM_META_DATA, Queue(), True)
    start_server(NODE_NAME)