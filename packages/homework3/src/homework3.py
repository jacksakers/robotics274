#!/usr/bin/env python3
#
import rospy
from std_msgs.msg import Float32

def homework3():
    pub = rospy.Publisher('delta', Float32, queue_size=10)
    rospy.init_node('homework3', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(69)
        rate.sleep()

if __name__ == '__main__':
    try:
        homework3()
    except rospy.ROSInterruptException:
        pass

