#!/usr/bin/env python3

import unittest
from unittest.mock import patch, MagicMock
from multiprocessing import Process
from robot import Robot

class TestRobot(unittest.TestCase):
    @patch('robot.ComponentFactory')
    def test_init_components(self, mock_component_factory):
        mock_component_factory.return_value = {0 : MagicMock(), 1 : MagicMock(), 2 : MagicMock()}

        robot = Robot()

        self.assertEqual(len(robot.processes), 3)
        self.assertEqual(len(robot.queues), 3)
        for process in robot.processes:
            self.assertTrue(isinstance(process, Process))

if __name__ == '__main__':
    unittest.main()