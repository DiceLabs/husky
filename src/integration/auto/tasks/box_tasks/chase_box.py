from dexterity import Dexterity
from arm_client import call_arm
from client import call_service
from services import ServiceNames, ServicePorts
from defaults import Defaults

def chase_box():
    response = call_service(port=ServicePorts[ServiceNames.RIGHT_CAMERA], request=Defaults.Trigger)
    print(response)
    if not response == None and response != (0.0, -0.0, 0.0):
        depth, dx, dy = response
        call_arm(dexterity=Dexterity.RIGHT, function="change_pose_ros", args={
            "x": f"{depth-.2}",
            "y": f"{dx}",
            "z": f"{dy}",
            "yaw":      "0",
            "pitch":    "0",
            "roll":     "0",
            "blocking": "True"})
        exit()
