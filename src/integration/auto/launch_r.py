#!/usr/bin/python3

import rospy
from client import call_service
from services import ServiceNames, ServicePorts
from defaults import Defaults
from req_resp import GenericRequest
from robot_types import Position, Euler, Quaternion
from conversions import euler_to_quaternion

def chase_box():
    r_response = call_service(port=ServicePorts[ServiceNames.RIGHT_CAMERA], request=Defaults.Trigger)
    print(r_response)
    if not r_response == None and r_response != (0.0, 0.0, 0.0):
        depth, dx, dy = r_response
        call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
            function="change_pose_rel", 
            args={
                "blocking":True,
                "position": Position(depth-.2, dx, dy),
                "orientation": Euler()
            }))
        
    l_response = call_service(port=ServicePorts[ServiceNames.LEFT_CAMERA], request=Defaults.Trigger)
    print(l_response)
    if not l_response == None and l_response != (0.0, 0.0, 0.0):
        depth, dx, dy = l_response
        call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(
            function="change_pose_rel", 
            args={
                "blocking":True,
                "position": Position(depth-.2, dx, dy),
                "orientation": Euler()
            }))


def grab_box():
    call_service(port=ServicePorts[ServiceNames.LEFT_GRIPPER],  request=GenericRequest(function="close", args={}))
    call_service(port=ServicePorts[ServiceNames.RIGHT_GRIPPER], request=GenericRequest(function="close", args={}))


def r_absolute_move():
    w, x, y, z = euler_to_quaternion(yaw=0,pitch=0, roll=0)
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="change_pose_abs", 
        args={ 
        "position": Position(x=0.5,y=-0.6,z=.2), 
        "orientation": Quaternion(w=w,x=x,y=y,z=z),
        "blocking": True
    }))

def l_absolute_move():
    call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(
        function="change_pose_abs", 
        args={ 
        "position": Position(x=0.6,y=0.6,z=.2), 
        "orientation": Quaternion(x=0.7686640998401458,y=0.019500606118823264,z=-0.008383110028113847,w=0.6393003608979063),
        "blocking": True
    }))


def turn_grippers():
    call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(
        function="change_pose_rel", 
        args={
            "blocking":False,
            "position": Position(),
            "orientation": Euler(yaw=10, roll=0, pitch=-20)
        }))
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="change_pose_rel", 
        args={
            "blocking":False,
            "position": Position(),
            "orientation": Euler(yaw=10, roll=0, pitch=20)
        }))



""" x: 0.8716764745489672
  y: -0.39605060704304607
  z: 0.4689581588137493 """

"""  x: 0.7996973773969469
  y: -0.3820427122373401
  z: 0.7881975376919993
 """

""" x: 0.3517103402232481
  y: 0.739102593420769
  z: 1.1694511999029744
 """


def relative_move():
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="move_up", 
        args={ 
        "blocking": True
    }))

def r_get_info():
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="print_info", 
        args={}
    ))

def l_get_info():
    call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(
        function="print_info", 
        args={}
    ))


def lift_arms():
    call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
        function="change_pose_rel", 
        args={
            "blocking":False,
            "position": Position(z=1),
            "orientation": Euler()
        }))
        
    call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(
        function="change_pose_rel", 
        args={
            "blocking":True,
            "position": Position(z=1),
            "orientation": Euler()
        }))


NODE_NAME = 'launch'
if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    grab_box()
    # lift_arms()
    
    # turn_grippers()
    # chase_box()
    # get_info()
    # left_get_info()
    # l_absolute_move()
    # r_absolute_move()
    # relative_move()
    # rospy.signal_shutdown()



""" 
    Good Side State Right
        x: 0.7981978650345217
        y: -0.6342592778018695
        z: 0.1845239201239313
    orientation: 
        x: -0.028026025935007363
        y: -0.7639695300163926
        z: -0.6437651947236414
        w: 0.03363737711615717

"""

""" 
    Good Side State Left
        x: 0.8050852069144842
        y: 0.6918660664995203
        z: 0.19128835340205522
    orientation: 
        x: 0.7686640998401458
        y: 0.019500606118823264
        z: -0.008383110028113847
        w: 0.6393003608979063

"""