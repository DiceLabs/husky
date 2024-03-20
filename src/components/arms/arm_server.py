#!/usr/bin/env python
import rospy
from args import convert_dext_str_to_enum
from dexterity import Dexterity
from arms import UR5e_Arm


def handle_arm_request(req):
    
    return AddTwoIntsResponse(req.a + req.b)


CALLBACK_FUNCTION = handle_arm_request
MESSAGE_TYPE = None
SERVICE_NAME = None
def start_server():
    arm = UR5e_Arm(dexterity)
    rospy.Service(SERVICE_NAME, MESSAGE_TYPE, lambda: CALLBACK_FUNCTION(arm))
    rospy.spin()

NODE_SUFFIX = 'arm'
DEXT_ARG  = '~dexterity'
if __name__ == "__main__" :
    dexterity_arg: str = rospy.get_param(DEXT_ARG)
    dexterity: Dexterity = convert_dext_str_to_enum(dexterity_arg)
    dexterity_name: str = dexterity.__str__()
    rospy.init_node(dexterity_name+NODE_SUFFIX, anonymous=True)
    