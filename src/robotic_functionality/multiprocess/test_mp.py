#!/usr/bin/env python3

import unittest
from unittest.mock import patch, MagicMock
from multiprocessing import Process, Queue
from robot import Robot, start_generic_component
from components import ComponentId

class TestRobot(unittest.TestCase):
    @patch('robot.ComponentFactory')
    def test_init_components(self, mock_component_factory):
        mock_component_factory.return_value = {0 : MagicMock(), 1 : MagicMock(), 2 : MagicMock()}

        robot = Robot()

        self.assertEqual(len(robot.processes), 3)
        self.assertEqual(len(robot.queues), 3)
        for process in robot.processes:
            self.assertTrue(isinstance(process, Process))

    @patch('robot.start_generic_component')
    def test_generic_components(self, mock_generic):
        # mock_generic.return_value = 
        mock = MagicMock()
        queue = Queue()
        kwargs = {"componentId" : ComponentId.BASE, "component": mock, "messageQueue": queue}
        component = start_generic_component(**kwargs)

        self.assertEqual(component.componentId, ComponentId.BASE)
        self.assertEqual(component.component, mock)
        self.assertEqual(component.messageQueue, queue)

    def test_generic_message(self):
        pass

if __name__ == '__main__':
    unittest.main()