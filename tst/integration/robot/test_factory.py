#!/usr/bin/env python3

from unittest.mock import patch, call
import unittest
from factory import ComponentFactory
from dexterity import Dexterity

class TestComponentFactory(unittest.TestCase):
    @patch('factory.BaseNode')
    @patch('factory.UR5e_Arm')
    @patch('factory.GripperNode')
    def test_ComponentFactory(self, MockGripperNode, MockUR5e_Arm, MockBaseNode):
        ComponentFactory()
        
        MockBaseNode.assert_called_once()
        calls_ur5e = [call(dexterity=Dexterity.LEFT), call(dexterity=Dexterity.RIGHT)]
        calls_gripper = [call(dexterity=Dexterity.LEFT), call(dexterity=Dexterity.RIGHT)]
        MockUR5e_Arm.assert_has_calls(calls_ur5e, any_order=True)
        MockGripperNode.assert_has_calls(calls_gripper, any_order=True)
        
        self.assertEqual(MockUR5e_Arm.call_count, 2)
        self.assertEqual(MockGripperNode.call_count, 2)

if __name__ == "__main__":
    unittest.main()