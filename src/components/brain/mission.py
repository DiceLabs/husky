""" 
    Configuration file for Robot Tasks
"""
from dataclasses import dataclass
from typing import List, Callable
from brain import Brain

@dataclass
class Transition():
    condition: bool
    state    : int

@dataclass
class Task():
    current_state   : int
    end_condition   : bool
    state_actions   : List[ Callable[[Brain], None] ]
    transitions     : List[Transition]
