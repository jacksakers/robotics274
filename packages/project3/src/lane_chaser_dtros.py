#!/usr/bin/env python3

import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import FSMState
from time import sleep

class LaneChaser(DTROS):
    def __init__(self, node_name):
        super(LaneChaser, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

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
    lane_controller_node = LaneChaser(node_name="lane_chaser")
    rospy.spin()
