#!/usr/bin/python3

import rospy
from brain import Brain
from mission import tasks

class Context():
    FREQUENCY = 20
    NODE_NAME = 'launch'
    PERIOD = 1/FREQUENCY

if __name__ == "__main__":
    tasklist = tasks()
    brain = Brain(tasklist)
    rospy.init_node(Context.NODE_NAME, anonymous=True)
    timer = rospy.Timer(rospy.Duration(Context.PERIOD), brain.execute())
    rospy.spin()
