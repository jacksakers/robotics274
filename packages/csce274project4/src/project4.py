#!/usr/bin/env python3
import numpy as np
import rospy
import time

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    FSMState,
    StopLineReading,
)

from my_controller_attempt import LaneController


class LaneControllerNode(DTROS):
    
    def __init__(self, node_name):

        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)


        self.controller = LaneController()

        self.fsm_state = None
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.last_s = None
        

        self.current_pose_source = "lane_filter"

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher(
            "/duck32/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber(
            "/duck32/lane_filter_node/lane_pose", LanePose, self.cbAllPoses, "lane_filter", queue_size=1
        )
        self.sub_wheels_cmd_executed = rospy.Subscriber(
            "~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmdExecuted, queue_size=1
        )
        
        self.sub_state = rospy.Subscriber(
            "/duck32/fsm_node/mode", FSMState, self.cbMode, queue_size=1
        )
        rospy.logerr("Jack Sakers is Initialized!")



    

    def cbMode(self, fsm_state_msg):
 
        self.fsm_state = fsm_state_msg.state  # String of current FSM state
        if self.fsm_state == "LANE_FOLLOWING":
            self.checker = 1
        else:
            self.checker = 0
        self.current_pose_source = "lane_filter"

        if 2 == 2:
            self.log("Pose source: %s" % self.current_pose_source)

    def cbAllPoses(self, input_pose_msg, pose_source):
        self.pose_msg = input_pose_msg

        self.getControlAction(self.pose_msg)

    def cbWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd

    def publishCmd(self, car_cmd_msg):
        if self.checker == 1:
            self.pub_car_cmd.publish(car_cmd_msg)
            rospy.logerr("p: %s" % rospy.get_param('/duck32/project4/p', None))
            rospy.logerr("i: %s" % rospy.get_param('/duck32/project4/i', None))
            rospy.logerr("d: %s" % rospy.get_param('/duck32/project4/d', None))
            rospy.logerr("Duck: %s" % rospy.get_param('/duck32/project4/duck', None))
            rospy.logerr("vel_min: %s" % rospy.get_param('/duck32/project4/vel_min', None))
            rospy.logerr("vel_max: %s" % rospy.get_param('/duck32/project4/vel_max', None))

    def getControlAction(self, pose_msg):
        current_s = rospy.Time.now().to_sec()
        dt = None
        if self.last_s is not None:
            dt = current_s - self.last_s

        d_err = pose_msg.d
        phi_err = pose_msg.phi
        

        if np.abs(d_err) > 5:
            self.log("d_err too large, thresholding it!", "error")
            d_err = np.sign(d_err) * 5
        wheels_cmd_exec = [1,1]
        rospy.logerr("vel_left: %s" % self.v_left)
        rospy.logerr("vel_right: %s" % self.v_right)
        self.v_right = rospy.get_param('/duck32/project4/vel_max', None)
        self.v_left = self.controller.compute_control_action(
           d_err, phi_err, dt, wheels_cmd_exec
        )

        # Initialize car control msg, add header from input message
        car_control_msg = WheelsCmdStamped(None, self.v_left, self.v_right)
        
        self.publishCmd(car_control_msg)
        self.last_s = current_s

if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name="project4")
    # Keep it spinning
    rospy.spin()
