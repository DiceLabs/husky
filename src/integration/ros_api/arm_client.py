import rospy
from factory import ComponentId
from generic_ros_client import call_generic_service
from dexterity import Dexterity


LEFT_ARM_SERVICE = '/left_ur/left_arm'
RIGHT_ARM_SERVICE = '/right_ur/right_arm'
LEFT_ARM_ID = ComponentId.LEFT_ARM
RIGHT_ARM_ID = ComponentId.RIGHT_ARM


def call_arm(dexterity: Dexterity, function: str, args: dict):
    if dexterity == Dexterity.LEFT:
        call_generic_service(service_name=LEFT_ARM_SERVICE, component_id=LEFT_ARM_ID, function_name=function, args=args)
    if dexterity == Dexterity.RIGHT:
        call_generic_service(service_name=RIGHT_ARM_SERVICE, component_id=RIGHT_ARM_ID, function_name=function, args=args)


""" 
    LIVE_ACTION

    from arm_client import call_arm
    import rospy
    from factory import ComponentId
    from generic_ros_client import call_generic_service
    from dexterity import Dexterity
    call_arm(Dexterity.LEFT, "move_down", {}) 
"""

# Example with bad coding practices but it's clear
if __name__ == "__main__":
    NODE_NAME = 'arm_client'
    rospy.init_node(NODE_NAME, anonymous=True)
    from time import sleep
    MOVE_DOWN = "move_down"
    MOVE_UP = "move_up"
    ARGS = {}
    i = 0

    while i < 5:
        call_arm(Dexterity.LEFT, MOVE_DOWN, ARGS)
        call_arm(Dexterity.RIGHT, MOVE_DOWN, ARGS)
        sleep(2.2)
        call_arm(Dexterity.LEFT, MOVE_UP, ARGS)
        call_arm(Dexterity.RIGHT, MOVE_UP, ARGS)
        sleep(2.2)