#!/usr/bin/python3

from mission import Task
from queue import Queue

class Brain():
    """ 
        Runs program main loop - 
            - Updates at given frequency 
            - With current desired task
                - Analyze needed sensors 
                - Give appropriate components
    """
    def __init__(self, mission: "Queue[Task]"=[]):
        self.mission    = mission
        self.task       = mission.pop()
    def execute(self):
        if self.task.end_condition:
            self.next_task()
            return
        self.task.state_actions[self.task.current_state]()
        for transition in self.task.transitions:
            if transition.condition:
                self.task.current_state = transition.state
    def next_task(self):
        self.task       = self.mission.pop()
