#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from time import sleep
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    FSMState,
    StopLineReading,
)

class Project69:
    def __init__(self):
        self.pub = rospy.Publisher('/duck32/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1) #CHANGE TO CORRECT TOPIC and type
        self.v_left = 0.0
        self.v_right = 0.0
        self.firstpause = rospy.get_param('firstpause', None)
        self.secondpause = rospy.get_param('secondpause', None)
        self.talker()
        sleep(4)
        for x in range(3):
          rospy.loginfo("Going Straight")
          self.v_left = 0.5
          self.v_right = 0.5
          self.talker()
          sleep(2.5)
          rospy.loginfo("Turning")
          self.v_left = 0.2
          self.v_right = 0.5
          self.talker()
          sleep(4)
          rospy.loginfo("Looping")
        self.v_left = 0
        self.v_right = 0
        self.talker()

    def talker(self):
        msg = WheelsCmdStamped(None, self.v_left, self.v_right) #change to correct type
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('project69')
    Project69()
