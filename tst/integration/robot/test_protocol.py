#!/usr/bin/env python3

from protocol import GenericComponent
from factory import ComponentId
from robot import RobotMessage
from unittest.mock import patch, MagicMock
from multiprocessing import Queue
import unittest

class TestGenericComponent(unittest.TestCase):
    def setUp(self):
        self.component_id = ComponentId.BASE
        self.component = MagicMock()
        self.message_queue = Queue()
        self.listen_patch = patch.object(GenericComponent, 'listen')
        self.mock_listen = self.listen_patch.start()
        self.generic_component = GenericComponent(self.component_id, self.component, self.message_queue)

    def test_handle_message_calls_callback(self):
        message = RobotMessage(componentId=self.component_id, function="test_function", data={})
        self.component.test_function = MagicMock()

        self.message_queue.put(message)
        self.generic_component.handle_message(message)

        self.component.test_function.assert_called_once_with()

    def test_nonexistent_function(self):
        message = RobotMessage(componentId=self.component_id, function="misspelled_function", data={})
        self.component.test_function = MagicMock()

        self.message_queue.put(message)
        self.generic_component.handle_message(message)

        self.component.test_function.assert_not_called()

    def test_not_intended_system(self):
        message = RobotMessage(componentId=ComponentId.RIGHT_ARM, function="test_function", data={})
        self.component.test_function = MagicMock()

        self.message_queue.put(message)
        self.generic_component.handle_message(message)

        self.component.test_function.assert_not_called()

    def test_handle_message_calls_with_args_callback(self):
        message = RobotMessage(componentId=self.component_id, function="test_function", data={"arg" : 5})
        self.component.test_function = MagicMock()

        self.message_queue.put(message)
        self.generic_component.handle_message(message)

        self.component.test_function.assert_called_once_with(arg=5)
    
    
if __name__ == "__main__":
    unittest.main()