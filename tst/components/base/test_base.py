#!/usr/bin/env python3

from base import BaseNode
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import unittest

def test_create_base_msg():
    base = BaseNode()
    lin_vel = Vector3(0, 0, 0)
    ang_vel = Vector3(0.1, 0, 0)

    msg = base.create_base_msg(lin_vel, ang_vel)

    expected_msg = Twist()
    expected_msg.linear = lin_vel
    expected_msg.angular = ang_vel

    assert expected_msg == msg

if __name__ == '__main__':
    unittest.main()