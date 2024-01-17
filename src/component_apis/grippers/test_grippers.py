import unittest
from unittest.mock import patch, MagicMock
from grippers import GripperNode, close_gripper, open_gripper, MessageFiller, on_key_press
from pynput import keyboard
from pynput.keyboard import Key

class TestGripperNode(unittest.TestCase):

    @patch('grippers.rospy.close_gripper')
    def test_close_gripper(self, mock_publisher):
        gripper_node = GripperNode()
        message_filler = MessageFiller()

        # Mock the pub method of the publisher
        mock_pub = MagicMock()
        mock_publisher.return_value.pub = mock_pub

        # Call the close_gripper function
        close_gripper(message_filler, gripper_node.left_pub)

        # Assert that pub method was called with the correct arguments
        mock_pub.assert_called_once_with(message_filler.create_msg(0.00))

    @patch('grippers.open_gripper')
    def test_open_gripper(self, mock_publisher):
        gripper_node = GripperNode()
        message_filler = MessageFiller()

        # Mock the pub method of the publisher
        mock_pub = MagicMock()
        mock_publisher.return_value.pub = mock_pub

        # Call the open_gripper function
        open_gripper(message_filler, gripper_node.left_pub)

        # Assert that pub method was called with the correct arguments
        mock_pub.assert_called_once_with(message_filler.create_msg(0.085))


def test_on_key_press():
    gripper = GripperNode()
    key = Key.A
    on_key_press(key, gripper)

if __name__ == '__main__':
    unittest.main()
