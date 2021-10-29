#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo("Meters: %s", converter(data))
    try:
        talker(converter(data))
    except:
        pass
    
def singlenode():
    rospy.init_node('singlenode')
    listener()

def talker(result):
    pub = rospy.Publisher('/homework5/converted', Float32, queue_size=10)
    rate = rospy.Rate(10)
    pub.publish(result)

def listener():
    rospy.Subscriber('/homework2/total', Float32, callback)
    rospy.spin()

def converter(number):
    meters = (number/3.2808)
    return meters

if __name__ == '__main__':
    try:
        singlenode()
    except rospy.ROSInterruptException:
        rospy.loginfo("BIG ERROR")
