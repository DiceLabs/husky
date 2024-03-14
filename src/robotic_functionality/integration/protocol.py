#!/usr/bin/env python3

from components import ComponentId, ComponentFactory
from dataclasses import dataclass
from multiprocessing import Queue
from typing import Dict

"""
    Generic Listener just has to take command and call the appropriate function
    Send command over general bus
    Each component will listen on bus and react to pertinet commands
    Data field will have the args to pass into the function
    Generic component will have access to a queue
"""

@dataclass
class RobotMessage():
    componentId: ComponentId
    function: str
    data: Dict

class GenericComponent():
    def __init__(self, componentId: ComponentId, component, messageQueue: 'Queue[RobotMessage]'):
        self.componentId = componentId
        self.component = component
        self.command_queue = messageQueue
        self.listen()

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
        while True:
            message = self.command_queue.get()
            self.handle_message(message)