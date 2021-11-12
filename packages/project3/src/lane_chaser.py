#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import Pose2DStamped
from time import sleep

class PIDCalc:
    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0):
        self.kP = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.integral = 0
        self.dt = .1;
        self.set_point = 0.0
        self.error = 0.0
    def update(self.measured_value):
        self.error = self.set_point - self.measured_value
        integral = self.integral + self.error + self.dt
        self.Derivative = (self.error - self.previous_error) / self.dt
        self.previous_error = self.error
        PID = self.kP * self.error + self.ki * self.integral + self.Kd * self.Derivative
        return PID

class talker:
    def _init_(self):
        sub = rospy.Subscriber('lane_controller_node/lane_post', Pose2DStamped, self.pose_callback)
        pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        rospy.init_node('lane_chaser', anonymous=True)
        rate = rospy.Rate(10)
    def talk():
        __init__()
        vel, o = update(sub)
        msg = Twist2DStamped(header=None, v=vel, omega=o)
        pub.publish(msg)
if __name__ == '__main__':
    try:
        _init_(self)
        while not rospy.is_shutdown():
            talk()
            rospy.logerror("Jack Sakers Lane Following Node")
    except rospy.ROSInterruptException:
        pass
