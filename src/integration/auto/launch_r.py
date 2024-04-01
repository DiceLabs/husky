#!/usr/bin/python3

import rospy
from client import call_service
from services import ServiceNames, ServicePorts
from defaults import Defaults
from req_resp import GenericRequest
from robot_types import Position, Euler, Quaternion

def chase_box():
    r_response = call_service(port=ServicePorts[ServiceNames.RIGHT_CAMERA], request=Defaults.Trigger)
    print(r_response)
    if not r_response == None and r_response != (0.0, 0.0, 0.0):
        depth, dx, dy = r_response
        call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
            function="change_pos_rel", 
            args={
                "blocking":True,
                "position": Position(depth-.2, dx, dy),
                "euler": Euler()
            }))
        
    # l_response = call_service(port=ServicePorts[ServiceNames.LEFT_CAMERA], request=Defaults.Trigger)
    # print(l_response)
    # if not l_response == None and l_response != (0.0, 0.0, 0.0):
    #     depth, dx, dy = l_response
    #     call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(
    #         function="change_pos_rel", 
    #         args={
    #             "blocking":True,
    #             "position": Position(depth-.2, dx, dy),
    #             "euler": Euler()
    #         }))

def grab_box():
    call_service(port=ServicePorts[ServiceNames.LEFT_GRIPPER],  request=GenericRequest(function="close", args={}))
    call_service(port=ServicePorts[ServiceNames.RIGHT_GRIPPER], request=GenericRequest(function="close", args={}))


def absolute_move():
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="change_pos_abs", 
        args={ 
        "position": Position(x=0.3,y=0.6,z=1.2), 
        "blocking": True
    }))

def relative_move():
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="move_up", 
        args={ 
        "blocking": True
    }))

def get_info():
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="print_info", 
        args={}
    ))

NODE_NAME = 'launch'
if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    # chase_box()
    # get_info()
    # grab_box()
    relative_move()
    # rospy.signal_shutdown()
