#!/usr/bin/env python3

from protocol import GenericComponent, RobotMessage
from multiprocessing import Process, Queue
from components import ComponentFactory, ComponentId
from typing import List

""" 
    Start a process for the base
    Start a process for each gripper
    Start a process for each arm
    Each process can be sent message from parent
    That will encourage each system to perform appropriate action
"""

COMPONENT_ID_ARG   = "componentId"
COMPONENT_ARG      = "component"
MSG_QUEUE_ARG      = "messageQueue"

""" Wrapper to instantiate class with kwargs in Process API """
def start_generic_component(**config):
    return GenericComponent(**config)

class Robot():
    def __init__(self):
        self.processes  : List[Process] = []
        self.queues     : List[Queue]   = []
        self.           init_components()
        
    def init_components(self):
        componentFactory = ComponentFactory()
        for componentId, component in componentFactory.items():
            componentQueue = Queue()
            componentArgs = {
                COMPONENT_ID_ARG: componentId, 
                COMPONENT_ARG   : component,
                MSG_QUEUE_ARG   : componentQueue
            }
            component_process = Process(target=start_generic_component, kwargs=componentArgs)
            self.processes.append(component_process)
            self.queues.append(componentQueue)

    def send_message(self, message: RobotMessage):
        for queue in self.queues:
            queue.put(message)

    def start(self):
        for process in self.processes:
            process.start()
        
    def kill(self):
        for process in self.processes:
            process.kill()

if __name__ == "__main__":
    robot = Robot()