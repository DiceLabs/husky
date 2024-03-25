#!/usr/bin/env python3

from protocol import GenericComponent, GenericComponentMetaData
from factory import ComponentId
from robot import RobotMessage
from unittest.mock import patch, MagicMock
from multiprocessing import Queue
import unittest

class TestGenericComponent(unittest.TestCase):
    def setUp(self):
        self.component_id = ComponentId.BASE
        self.component = GenericComponentMetaData("mock", MagicMock, {}) 
        self.message_queue = Queue()
        self.listen_patch = patch.object(GenericComponent, 'listen')
        self.ros_patch = patch.object(GenericComponent, 'init_ros')
        self.mock_ros = self.ros_patch.start()
        self.mock_listen = self.listen_patch.start()
        self.generic_component = GenericComponent(self.component_id, self.component, self.message_queue, False)

    def test_handle_message_calls_callback(self):
        message = RobotMessage(componentId=ComponentId.BASE, function="test_function", data={})
        self.component.component.test_function = MagicMock()

        self.message_queue.put(message)
        self.generic_component.handle_message(message)

        self.component.component.test_function.assert_called_once_with()

    def test_nonexistent_function(self):
        message = RobotMessage(componentId=ComponentId.BASE, function="misspelled_function", data={})
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
        message = RobotMessage(componentId=ComponentId.BASE, function="test_function", data={"arg":5})
        self.component.component.test_function = MagicMock()

        self.message_queue.put(message)
        self.generic_component.handle_message(message)

        self.component.component.test_function.assert_called_once_with(arg=5) 
    
    def test_listen(self):        
        self.component.test_function = MagicMock()
        test_message = RobotMessage(componentId=ComponentId.BASE, function="test_function", data={})
        self.message_queue.put(test_message)
        message = self.generic_component.command_queue.get()
        self.assertEqual(message, test_message)

    def tearDown(self) -> None:
        self.listen_patch.stop()
        self.ros_patch.stop()
        self.generic_component.timer.stop()

if __name__ == "__main__":
    unittest.main()