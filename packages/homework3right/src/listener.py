#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo("Total: %s", data.data)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('/homework2/total', Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
