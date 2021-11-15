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

from my_controller import LaneController


class LaneControllerNode(DTROS):
    
    def __init__(self, node_name):

        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.params = dict()
        self.params["~v_bar"] = DTParam("~v_bar", param_type=ParamType.FLOAT, min_value=0.0, max_value=5.0)
        self.params["~k_d"] = DTParam("~k_d", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_theta"] = DTParam(
            "~k_theta", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        self.params["~k_Id"] = DTParam("~k_Id", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_Iphi"] = DTParam(
            "~k_Iphi", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        self.params["~theta_thres"] = rospy.get_param("~theta_thres", None)
        self.params["~d_thres"] = rospy.get_param("~d_thres", None)
        self.params["~d_offset"] = rospy.get_param("~d_offset", None)
        self.params["~integral_bounds"] = rospy.get_param("~integral_bounds", None)
        self.params["~d_resolution"] = rospy.get_param("~d_resolution", None)
        self.params["~phi_resolution"] = rospy.get_param("~phi_resolution", None)
        self.params["~omega_ff"] = rospy.get_param("~omega_ff", None)
        self.params["~verbose"] = rospy.get_param("~verbose", None)

        self.controller = LaneController(self.params)

        self.fsm_state = None
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.last_s = None
        

        self.current_pose_source = "lane_filter"

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher(
            "/duck32/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
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
        self.log("Jack Sakers is Initialized!")



    

    def cbMode(self, fsm_state_msg):
 
        self.fsm_state = fsm_state_msg.state  # String of current FSM state
        if self.fsm_state == "LANE_FOLLOWING":
            self.checker = 1
        else:
            self.checker = 0
        self.current_pose_source = "lane_filter"

        if self.params["~verbose"] == 2:
            self.log("Pose source: %s" % self.current_pose_source)

    def cbAllPoses(self, input_pose_msg, pose_source):
        self.pose_msg = input_pose_msg

        self.getControlAction(self.pose_msg)

    def cbWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd

    def publishCmd(self, car_cmd_msg):
        if self.checker == 1:
            self.pub_car_cmd.publish(car_cmd_msg)
            rospy.loginfo("JACK SAKERS LANE FOLLOWER IS WORKING!")

    def getControlAction(self, pose_msg):
        current_s = rospy.Time.now().to_sec()
        dt = None
        if self.last_s is not None:
            dt = current_s - self.last_s

        d_err = pose_msg.d - self.params["~d_offset"]
        phi_err = pose_msg.phi
        self.log("d: %s" % pose_msg.d)
        self.log("phi: %s" % pose_msg.phi)

        if np.abs(d_err) > self.params["~d_thres"]:
            self.log("d_err too large, thresholding it!", "error")
            d_err = np.sign(d_err) * self.params["~d_thres"]
        wheels_cmd_exec = [1,1]
        
         
        v, omega = self.controller.compute_control_action(
           d_err, phi_err, dt, wheels_cmd_exec
        )

        self.log("v: %s" % v)
        self.log("omega: %s" % omega)

        # Initialize car control msg, add header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        # Add commands to car message
        car_control_msg.v = v
        car_control_msg.omega = omega

        self.publishCmd(car_control_msg)
        self.last_s = current_s

    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name="new_lane")
    # Keep it spinning
    rospy.spin()
