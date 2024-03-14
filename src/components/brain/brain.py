#!/usr/bin/python3

from robot  import Robot, Sensors
from mission import Task
from typing import List

UPDATE_FREQUENCY     = 50
MISSION_COMPLETE_MSG = "Mission Complete"

class Brain():
    """ 
        Runs program main loop - 
            - Updates at given frequency 
            - With current desired task
                - Analyze needed sensors 
                - Give appropriate components
    """
    def __init__(self, sensors: Sensors, components: Robot, mission: List[Task]=[]):
        self.sensors    = sensors
        self.components = components
        self.mission    = mission
    def run(self):
        for task in self.mission:
            self.execute(task)
        self.components.kill()
        self.sensors   .kill()
        print(MISSION_COMPLETE_MSG)
    def execute(self, task: Task):
        while not task.end_condition:
            task.state_actions[task.current_state](self.sensors, self.components)
            for transition in task.transitions:
                if transition.condition:
                    task.current_state = transition.state

