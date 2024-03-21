#!/usr/bin/env python3

import rospy
from factory import ComponentId
from robot_msgs.srv import GenericSrv, GenericSrvRequest
from robot_msgs.msg import KeyValue

def create_generic_request(component_id: ComponentId, function_name: str, args: dict):
    """
    Create a GenericSrvRequest message with specified component ID, function name, and arguments.
    Arguments should be a dictionary where keys are argument names and values are their values.
    """
    request = GenericSrvRequest()
    request.message.componentId = component_id.value
    request.message.function = function_name
    for arg_name, arg_value in args.items():
        arg = KeyValue()
        arg.key = arg_name
        arg.value = arg_value
        request.message.args.append(arg)
    return request

def call_generic_service(service_name :str, component_id: ComponentId, function_name: str, args: dict):
    # rospy.wait_for_service(service_name)
    try:
        arm_service = rospy.ServiceProxy(service_name, GenericSrv)
        request = create_generic_request(component_id, function_name, args)
        print(request)
        response = arm_service(request)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

