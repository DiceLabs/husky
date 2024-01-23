# Import necessary modules
from component_apis.base.goal_base import create_base_msg, LinearVelocity, AngularVelocity
from geometry_msgs.msg import Twist
import rospy

def test_create_base_msg():
    lin_vel = LinearVelocity(0, 0, 0)
    ang_vel = AngularVelocity(0.1, 0, 0)

    msg = create_base_msg(lin_vel, ang_vel)

    expected_msg = Twist()
    expected_msg.target_pose.pose.position = lin_vel
    expected_msg.target_pose.pose.orientation = ang_vel

    assert expected_msg == msg
