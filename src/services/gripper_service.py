#!/usr/bin/env python3

from multiprocessing import Process
from services import ServiceNames, ServicePorts
from generic_ros_server import start_ros_server
from grippers import GripperNode
from req_resp import GenericRequest
from dexterity import Dexterity
import rospy

def generic_callback(request: GenericRequest, gripper: GripperNode):
    function_call = getattr(gripper, request.function)
    function_call(**request.args)

def start_gripper_server(name, port, dexterity):
    rospy.init_node(name)
    gripper = GripperNode(dexterity)
    start_ros_server(name=name, port=port, callback=lambda request: generic_callback(request, gripper))

if __name__ == "__main__":
    left_process = Process(target=start_gripper_server, args=(ServiceNames.LEFT_GRIPPER, ServicePorts[ServiceNames.LEFT_GRIPPER], Dexterity.LEFT))
    right_process = Process(target=start_gripper_server, args=(ServiceNames.RIGHT_GRIPPER, ServicePorts[ServiceNames.RIGHT_GRIPPER], Dexterity.RIGHT))
    left_process.start()
    right_process.start()
    left_process.join()
    right_process.join()
