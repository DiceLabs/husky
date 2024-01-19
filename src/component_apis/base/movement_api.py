import rospy
from geometry_msgs import PoseStamped
from std_msgs.msg import Header

HUSKY_BASE_TOPIC = '/move_base_simple/goal geometry_msgs/PoseStamped'
FRAME_ID = 'base_link'
DEFAULT_SEQ_NUM = 0

class BaseNode():
    def __init__(self):
        self.base_pub = rospy.Publisher(HUSKY_BASE_TOPIC, PoseStamped, queue_size=10)

def fill_header(msg):
    msg.header = Header()
    msg.header.seq = DEFAULT_SEQ_NUM
    msg.header.stamp = rospy.Time(0)
    msg.header.frame_id = FRAME_ID

def create_base_msg(position, orientation):
    msg = PoseStamped()
    return msg

def fill_position(msg, x, y, z):
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z

def fill_orientation(msg, x, y, z, w):
    msg.orientation.x = x
    msg.orientation.y = y
    msg.orientation.z = z
    msg.orientation.w = w
