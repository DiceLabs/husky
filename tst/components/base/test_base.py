#!/usr/bin/env python3

from base import create_base_msg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def test_create_base_msg():
    lin_vel = Vector3(0, 0, 0)
    ang_vel = Vector3(0.1, 0, 0)

    msg = create_base_msg(lin_vel, ang_vel)

    expected_msg = Twist()
    expected_msg.linear = lin_vel
    expected_msg.angular = ang_vel

    assert expected_msg == msg
