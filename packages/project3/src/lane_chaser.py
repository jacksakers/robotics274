#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import FSMState
from time import sleep

class LaneChaser:
    def __init__(self):
        self.pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.lane_pose = rospy.Subscriber('/lane_filter_node/lane_pose', LanePose, self.listener)
        self.state = rospy.Subscriber('/fsm_node/mode', FSMState, self.listener)
        rospy.loginfo("HELLO??!!")
        self.talker()

    def listener(self, lane_data, state):
        self.input = lane_data.data
        self.state_info = state.data
        if self.state_info.state == "LANE_FOLLOWING":
            rospy.loginfo("JACK SAKERS LANE FOLLOWER IS WORKING!")
        
    def talker(self):
        msg = Twist2DStamped(header=None, v=1, omega=0)
        while not rospy.is_shutdown():
            if hasattr(self, 'input'):
                self.pub.publish(msg)
            sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('lane_chaser')
    while not rospy.is_shutdown():
        LaneChaser()
    rospy.spin()
