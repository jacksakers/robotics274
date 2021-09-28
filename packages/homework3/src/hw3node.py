#!/usr/bin/env python3

import rospy
import std_msgs.msg import Float32
import random

rospy.init_node('hw3node')
pub = rospy.Publisher('/homework2', Float32, queue_size=10)

pub.publish(1.02)
