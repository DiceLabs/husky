# Import necessary modules
from component_apis.base.goal_base import create_base_msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseGoal
import rospy
from std_msgs.msg import Header

def test_create_base_msg():
    header = Header()
    header.seq = 0
    header.stamp = rospy.Time(0)
    header.frame_id = 'base_link'
    position = Point(0, 0, 0)
    orientation = Quaternion(0.1, 0, 0, 0)

    msg = create_base_msg(position, orientation)

    expected_msg = MoveBaseGoal()
    expected_msg.target_pose.header = header
    expected_msg.target_pose.pose.position = position
    expected_msg.target_pose.pose.orientation = orientation

    assert expected_msg == msg
