#!/usr/bin/env python3

from factory import ComponentId, GenericComponentMetaData
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

@dataclass
class RobotMessage():
    componentId: ComponentId
    function: str
    data: Dict

class GenericComponent():
    def __init__(self, componentId: ComponentId, component: GenericComponentMetaData, messageQueue: 'Queue[RobotMessage]', rosMessaging: bool):
        self.componentId = componentId
        self.component = component.component(**component.args)
        self.command_queue = messageQueue
        if not rosMessaging:
            self.init_ros(component.name)
            self.init_timer()
    
    def init_ros(self, node_name):
        rospy.init_node(node_name)

    def init_timer(self):
        FREQUENCY = 20
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