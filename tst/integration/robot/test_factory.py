from unittest.mock import patch, MagicMock
import unittest
from factory import ComponentFactory
from dexterity import Dexterity

class TestComponentFactory(unittest.TestCase):
    @patch('base.BaseNode')
    @patch('arms.UR5e_Arm')
    @patch('grippers.GripperNode')
    def test_ComponentFactory(self, MockGripperNode, MockUR5e_Arm, MockBaseNode):
        ComponentFactory()
        MockBaseNode.assert_called_once()
        MockUR5e_Arm.assert_called_once_with(dexterity=Dexterity.LEFT)
        MockGripperNode.assert_called_once_with(dexterity=Dexterity.LEFT)

if __name__ == "__main__":
    unittest.main()