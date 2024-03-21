#!/usr/bin/env python

import rospy
from robot_msgs.srv import GenericSrv, GenericSrvRequest
from robot_msgs.msg import KeyValue
from factory import ComponentId

def create_generic_request(component_id, function_name, arguments):
    """
    Create a GenericSrvRequest message with specified component ID, function name, and arguments.
    Arguments should be a dictionary where keys are argument names and values are their values.
    """
    request = GenericSrvRequest()
    request.message.componentId = component_id
    request.message.function = function_name
    
    for arg_name, arg_value in arguments.items():
        arg = KeyValue()
        arg.key = arg_name
        arg.value = arg_value
        request.message.args.append(arg)
    
    return request

def call_generic_service(service_name, component_id, function_name, arguments):
    rospy.wait_for_service(service_name)
    try:
        arm_service = rospy.ServiceProxy(service_name, GenericSrv)
        request = create_generic_request(component_id, function_name, arguments)
        print(request)
        response = arm_service(request)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

NODE_NAME = 'arm_client'
LEFT_ARM_SERVICE = '/left_ur/left_arm'
RIGHT_ARM_SERVICE = '/right_ur/right_arm'
LEFT_ARM_ID = ComponentId.LEFT_ARM.value
RIGHT_ARM_ID = ComponentId.RIGHT_ARM.value
FUNCTION = "move_down"
FUNCTION2 = "move_up"
ARGS = {}

from time import sleep
if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    left_arm_response = call_generic_service(LEFT_ARM_SERVICE, LEFT_ARM_ID, FUNCTION, ARGS)
    right_arm_response = call_generic_service(RIGHT_ARM_SERVICE, RIGHT_ARM_ID, FUNCTION, ARGS)
    sleep(2.2)
    left_arm_response = call_generic_service(LEFT_ARM_SERVICE, LEFT_ARM_ID, FUNCTION2, ARGS)
    right_arm_response = call_generic_service(RIGHT_ARM_SERVICE, RIGHT_ARM_ID, FUNCTION2, ARGS)
    sleep(2.2)
    left_arm_response = call_generic_service(LEFT_ARM_SERVICE, LEFT_ARM_ID, FUNCTION, ARGS)
    right_arm_response = call_generic_service(RIGHT_ARM_SERVICE, RIGHT_ARM_ID, FUNCTION, ARGS)
    sleep(2.2)
    left_arm_response = call_generic_service(LEFT_ARM_SERVICE, LEFT_ARM_ID, FUNCTION2, ARGS)
    right_arm_response = call_generic_service(RIGHT_ARM_SERVICE, RIGHT_ARM_ID, FUNCTION2, ARGS)
