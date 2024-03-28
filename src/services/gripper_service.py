#!/usr/bin/env python3

from multiprocessing import Process
from services import ServiceNames, ServicePorts
from server import start_server
from grippers import GripperNode
from req_resp import GenericRequest

def generic_callback(request: GenericRequest, base: GripperNode):
    function_call = getattr(base, request.function)
    function_call(**request.args)

def start_gripper_server(name, port):
    gripper = GripperNode()
    start_server(name=name, port=port, callback=lambda request: generic_callback(request, gripper))

def start_gripper(service_name, service_port):
    process = Process(target=start_gripper_server, args=(service_name, service_port))
    process.start()
    process.join()

if __name__ == "__main__":
    start_gripper(ServiceNames.LEFT_GRIPPER,  ServicePorts[ServiceNames.LEFT_GRIPPER])
    start_gripper(ServiceNames.RIGHT_GRIPPER, ServicePorts[ServiceNames.RIGHT_GRIPPER])
