#!/usr/bin/env python

import rospy
from args import convert_dext_str_to_enum
from dexterity import Dexterity
from arms import UR5e_Arm
from robot_msgs.srv import GenericSrvRequest, GenericSrvResponse, GenericSrv
from protocol import GenericComponent, RobotMessage
from factory import ComponentId, GenericComponentMetaData
from multiprocessing import Queue

class Context():
    @staticmethod
    def get_dexterity_from_launch_arg():
        dexterity_arg: str = rospy.get_param(DEXT_LAUNCH_ARM)
        return convert_dext_str_to_enum(dexterity_arg)
    @staticmethod
    def create_node_name_from_dexterity(dexterity: Dexterity):
        node_suffix         = 'arm'
        dexterity_name: str = dexterity.__str__()
        return dexterity_name+node_suffix
    @staticmethod
    def convert_dexterity_to_component(dexterity: Dexterity):
        component_map = {
            Dexterity.LEFT : ComponentId.LEFT_ARM,
            Dexterity.RIGHT: ComponentId.RIGHT_ARM
        }
        return component_map[dexterity]
    @staticmethod
    def get_arm_meta_data(node_name: str, dexterity: Dexterity):
        DEXT_ARG_NAME       = "dexterity"
        args = {DEXT_ARG_NAME: dexterity}
        return GenericComponentMetaData(node_name, UR5e_Arm, args)


def arm_callback(request: GenericSrvRequest):
    args = {}
    for entry in request.message.args:
        args[entry.key] = entry.value
    robot_msg = RobotMessage(request.message.componentId, request.message.function, args)
    GENERIC_ARM.handle_message(robot_msg)
    return GenericSrvResponse()


def start_server(node_name: str):
    rospy.init_node(node_name, anonymous=True)
    rospy.Service(node_name, GenericSrv, arm_callback)
    rospy.spin()


DEXT_LAUNCH_ARM     = '~dexterity'
if __name__ == "__main__" :
    DEXTERITY: Dexterity = Context.get_dexterity_from_launch_arg()
    NODE_NAME: str = Context.create_node_name_from_dexterity(DEXTERITY)
    ARM_META_DATA: GenericComponentMetaData = Context.get_arm_meta_data(NODE_NAME)
    COMPONENT_ID: ComponentId = Context.convert_dexterity_to_component(DEXTERITY)
    GENERIC_ARM = GenericComponent(COMPONENT_ID, ARM_META_DATA, Queue(), True)
    start_server(NODE_NAME)