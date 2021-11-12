#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import LanePose
from time import sleep

class LaneChaser:
    def __init__(self):
        self.pub = rospy.Publisher('/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        rospy.Subscriber('/lane_filter_node/lane_pose', LanePose, self.listener)
        
    def listener(self, data):
        self.input = data.data
        rospy.loginfo("JACK SAKERS LANE FOLLOWER IS WORKING!")
        
    def talker(self):
        msg = Twist2DStamped(header=None, v=1, omega=0)
        while not rospy.is_shutdown():
            if hasattr(self, 'input'):
                self.pub.publish(msg)
            sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('lane_chaser')
    Homework5()
