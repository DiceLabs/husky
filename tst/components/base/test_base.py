#!/usr/bin/env python3

from base import BaseNode, LinearVelocity, AngularVelocity
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import unittest
from unittest.mock import MagicMock, patch

class TestBaseNode(unittest.TestCase):

    def setUp(self):
        self.base_node = BaseNode()

    def test_create_base_msg(self):
        lin_vel = LinearVelocity(0.1, 0, 0)
        ang_vel = AngularVelocity(0, 0, 0.1)
        msg = self.base_node.create_base_msg(lin_vel, ang_vel)
        self.assertEqual(msg.linear.x, lin_vel.x)
        self.assertEqual(msg.linear.y, lin_vel.y)
        self.assertEqual(msg.linear.z, lin_vel.z)
        self.assertEqual(msg.angular.x, ang_vel.x)
        self.assertEqual(msg.angular.y, ang_vel.y)
        self.assertEqual(msg.angular.z, ang_vel.z)

    def test_publish_base_message(self):
        with patch.object(self.base_node.base_pub, 'publish') as mock_publish:
            lin_vel = LinearVelocity(0.2, 0, 0)
            ang_vel = AngularVelocity(0, 0, 0)
            self.base_node.publish_base_message(lin_vel, ang_vel)
            mock_publish.assert_called_once()

    def test_clockwise(self):
        with patch.object(self.base_node.base_pub, 'publish') as mock_publish:
            self.base_node.CLOCKWISE()
            lin_vel = LinearVelocity(0, 0, 0)
            ang_vel = AngularVelocity(0, 0, 0.5)
            mock_publish.assert_called_with(self.base_node.create_base_msg(lin_vel, ang_vel))


if __name__ == '__main__':
    unittest.main()
