#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from time import sleep

class Homework5:
    def __init__(self):
        self.pub = rospy.Publisher('converter', Float32, queue_size=10)
        rospy.Subscriber('/homework2/total', Float32, self.listener)
        #rospy.spin()
        self.talker()

    def listener(self, data):
        self.input = data
        rospy.loginfo("Input: %s feet, Output: %s %s", data.data, self.converter(data), self.unit)

    
    def converter(self, data):
        self.unit = rospy.get_param('unit', "meters")
        if self.unit == "smoots":
            return data.data/5.5833
        elif self.unit == "feet":
            return data.data
        else:
            return data.data/3.2808

    def talker(self):
        while not rospy.is_shutdown():
            if hasattr(self, 'input'):
                self.pub.publish(self.converter(self.input))
            sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('homework5')
    Homework5()
