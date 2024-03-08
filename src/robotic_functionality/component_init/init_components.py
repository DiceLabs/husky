from protocol import GenericComponent, RobotMessage
from multiprocessing import Process, Queue
from components import ComponentFactory
from typing import List

""" 
    Start a process for the base
    Start a process for each gripper
    Start a process for each arm
    Each process can be sent message from parent
    That will encourage each system to perform appropriate action
"""

COMPONENT_ID_ARG   = "componentId"
MSG_QUEUE_ARG      = "messageQueue"

def start_generic_component(**config):
    return GenericComponent(**config)

class Robot():
    def __init__(self):
        self.processes  : List[Process] = []
        self.queues     : List[Queue]   = []
        self.           init_components()
        self.           start()
        
    def init_components(self):
        for componentId in ComponentFactory():
            componentQueue = Queue()
            componentArgs = {
                COMPONENT_ID_ARG: componentId, 
                MSG_QUEUE_ARG : componentQueue
            }
            component_process = Process(target=start_generic_component, args=componentArgs)
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