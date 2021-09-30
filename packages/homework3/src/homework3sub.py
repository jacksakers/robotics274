#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def homework3sub():
    rospy.init_node('homework3sub')

    rospy.Subscriber("total", Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        homework3sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

