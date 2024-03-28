""" 
    Configuration file for Robot Tasks
"""
from dataclasses import dataclass
from typing import List, Callable
from queue import Queue

@dataclass
class Transition():
    condition: bool
    state    : int

@dataclass
class Task():
    current_state   : int
    end_condition   : bool
    state_actions   : List[ Callable ]
    transitions     : List[Transition]

def tasks() -> "Queue[Task]":
    tasklist = Queue()
    ##############################
    """ User places Tasks here """
    
    """                        """
    ##############################
    return tasklist
