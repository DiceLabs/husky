#!/usr/bin/python3

from mission import Task
from enum import Enum
from brain import Brain

class SearchForBoxStates(Enum):
    SPOT_HANDLE = 0
    SPIN_LEFT   = 1

def box_found() -> bool:
    ...

def search(brain: Brain):
    brain.sensors


def search_for_box_actions():
    return []

def search_for_box_transitions():
    return []

def search_for_box():
    search_for_box = Task()
    search_for_box.current_state = SearchForBoxStates.SPOT_HANDLE
    search_for_box.end_condition = box_found()
    search_for_box.state_actions = search_for_box_actions()
    search_for_box.transitions   = search_for_box_transitions()
    return search_for_box