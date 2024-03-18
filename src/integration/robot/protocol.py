#!/usr/bin/env python3

from factory import ComponentId, GenericComponentInit
from dataclasses import dataclass
from multiprocessing import Queue
from typing import Dict
import rospy
from timer import Timer

"""
    Generic Listener just has to take command and call the appropriate function
    Send command over general bus
    Each component will listen on bus and react to pertinet commands
    Data field will have the args to pass into the function
    Generic component will have access to a queue
"""

FREQUENCY = 20

@dataclass
class RobotMessage():
    componentId: ComponentId
    function: str
    data: Dict

class GenericComponent():
    def __init__(self, componentId: ComponentId, component: GenericComponentInit, messageQueue: 'Queue[RobotMessage]'):
        rospy.init_node(component.name)
        self.componentId = componentId
        self.component = component.component(**component.args)
        self.command_queue = messageQueue
        self.timer = Timer(FREQUENCY, lambda: self.listen())
        self.timer.start()

    def handle_message(self, message: RobotMessage):
        FUNCTION_NOT_FOUND_ERROR = f"Function {message.function} not found in component {self.componentId.name}"
        if message.componentId != self.componentId:
            return
        try:
            callback = getattr(self.component, message.function)
            callback(**message.data)
        except AttributeError:
            print(FUNCTION_NOT_FOUND_ERROR)
        
    def listen(self):
        message = self.command_queue.get()
        self.handle_message(message)