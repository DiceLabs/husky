#!/usr/bin/python3

import rospy

from client import call_service
from services import ServiceNames, ServicePorts
from defaults import Defaults
from req_resp import GenericRequest
from robot_types import Position, Euler

def chase_box():
    response = call_service(port=ServicePorts[ServiceNames.RIGHT_CAMERA], request=Defaults.Trigger)
    print(response)
    if not response == None and response != (0.0, 0.0, 0.0):
        depth, dx, dy = response
        call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(
            function="change_pos_rel", 
            args={
                "blocking":True,
                "position": Position(depth-.2, dx, dy),
                "euler": Euler()
            }))
        rospy.signal_shutdown()

NODE_NAME = 'launch'
if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)
    chase_box()

