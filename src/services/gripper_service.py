#!/usr/bin/env python3

from multiprocessing import Process
from services import ServiceNames, ServicePorts
from generic_ros_server import start_ros_server
from grippers import GripperNode
from req_resp import GenericRequest
import rospy

def generic_callback(request: GenericRequest, base: GripperNode):
    function_call = getattr(base, request.function)
    function_call(**request.args)

def start_gripper_server(name, port):
    rospy.init_node(name)
    gripper = GripperNode()
    start_ros_server(name=name, port=port, callback=lambda request: generic_callback(request, gripper))

if __name__ == "__main__":
    left_process = Process(target=start_gripper_server, args=(ServiceNames.LEFT_GRIPPER, ServicePorts[ServiceNames.LEFT_GRIPPER]))
    right_process = Process(target=start_gripper_server, args=(ServiceNames.RIGHT_GRIPPER, ServicePorts[ServiceNames.RIGHT_GRIPPER]))
    left_process.start()
    right_process.start()
    left_process.join()
    right_process.join()
