from mission import Task
from enum import Enum
from robot import Sensors, Robot

class SearchForBoxStates(Enum):
    GO_TO_DEFAULT     = 0
    LOOk_UP_AND_DOWN  = 1
    CENTER_ON_HANDLE  = 2
    
def handle_found(sensors: Sensors, robot: Robot) -> bool:
    ...

def search(sensors: Sensors, robot: Robot):
    ...

def arm_search_for_box_actions(sensors: Sensors, robot: Robot):
    
    
    return []

def arm_search_for_box_transitions(sensors: Sensors, robot: Robot):
    return []

def arm_search_for_box(sensors: Sensors, robot: Robot):
    arm_search_for_box = Task()
    arm_search_for_box.current_state = SearchForBoxStates.SPOT_HANDLE
    arm_search_for_box.end_condition = handle_found(sensors, robot)
    arm_search_for_box.state_actions = arm_search_for_box_actions(sensors, robot)
    arm_search_for_box.transitions   = arm_search_for_box_transitions(sensors, robot)
    return arm_search_for_box