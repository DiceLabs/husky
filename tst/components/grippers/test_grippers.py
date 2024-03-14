import unittest
from unittest.mock import patch, MagicMock
from grippers import GripperNode, Dexterity, choose_gripper_topic, create_grip_msg, LEFT_GRIPPER_TOPIC, RIGHT_GRIPPER_TOPIC, CommandRobotiqGripperActionGoal

class TestGripperAPI(unittest.TestCase):

    def test_choose_gripper_topic(self):
        self.assertEqual(choose_gripper_topic(Dexterity.LEFT), LEFT_GRIPPER_TOPIC)
        self.assertEqual(choose_gripper_topic(Dexterity.RIGHT), RIGHT_GRIPPER_TOPIC)
        with self.assertRaises(SystemExit):
            choose_gripper_topic("invalid")

    @patch('grippers.rospy.Publisher')
    def test_GripperNode_init(self, mock_publisher):
        node = GripperNode(Dexterity.LEFT)
        mock_publisher.assert_called_with(LEFT_GRIPPER_TOPIC, CommandRobotiqGripperActionGoal, queue_size=10)
        self.assertIsNotNone(node.pub)

    @patch('grippers.publish_grip_message')
    @patch('grippers.GripperNode.__init__', return_value=None)
    def test_GripperNode_methods(self, mock_init, mock_publish):
        node = GripperNode(Dexterity.RIGHT)
        node.pub = MagicMock()

        node.close()
        mock_publish.assert_called_once_with(node.pub, 0.000)

        mock_publish.reset_mock()
        node.open()
        mock_publish.assert_called_once_with(node.pub, 0.085)

        mock_publish.reset_mock()
        node.half_grab()
        mock_publish.assert_called_once_with(node.pub, 0.085 / 2)

    @patch('grippers.fill_header')
    @patch('grippers.fill_goal_id')
    @patch('grippers.fill_goal')
    def test_create_grip_msg(self, mock_fill_goal, mock_fill_goal_id, mock_fill_header):
        msg = create_grip_msg(0.085)
        self.assertIsNotNone(msg)
        mock_fill_header.assert_called_once()
        mock_fill_goal_id.assert_called_once()
        mock_fill_goal.assert_called_once_with(msg, 0.085)

if __name__ == '__main__':
    unittest.main()
