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
        for x in range(6):
          self.v_left = 0.5
          self.v_right = 0.5
          self.talker()
          sleep(self.firstpause)
          self.v_left = 0.25
          self.v_right = 0.5
          self.talker()
          sleep(self.secondpause)

    def talker(self):
        msg = WheelsCmdStamped(None, self.v_left, self.v_right) #change to correct type
        while not rospy.is_shutdown():
          self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('project69')
    Project69()
