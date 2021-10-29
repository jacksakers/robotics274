#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import random

def talker():
    pub = rospy.Publisher('/homework2/delta', Float32, queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(random.randint(1,100))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
