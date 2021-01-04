#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import numpy as np
import math

import tf2_ros


from living_lab_robot_perception.msg import ReceiveTargetAction, ReceiveTargetFeedback, ReceiveTargetResult
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from qrcode_detector_ros.msg import Result
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Empty, String, Bool, Header


class OrderTargetServer:
    def __init__(self):
        print("init")

        self.start_detect = False
        self.detect_count = 0
        self.detect_done = False

        self.sub_order = rospy.Subscriber('order_target', String, self.handle_order)
        self.order_server = actionlib.SimpleActionServer('order_received', ReceiveTargetAction, self.handle_request_order, False)
        # self.sub_detect = rospy.Subscriber('detected_object', Result, self.handle_detector_result)
        # self.server = actionlib.SimpleActionServer('qrcode_detect', ReceiveTargetAction, self.handle_request_detect, False)

        self.order_server.start()
        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_order(self, msg):
        # print("Detector result!!!")
        if not self.start_detect:
            return

        print(msg)
        self.result_code_data = msg.data
        self.start_detect = False
        self.detect_done = True

        # print(self.result_orientation)

    def handle_request_order(self, goal):
        print("Request!!!")
        result = ReceiveTargetResult()
        success = True

        self.start_detect = True

        while(not self.detect_done):
            rospy.sleep(0.1)
        self.detect_done = False

        if success:
            result.result = True
            result.data = self.result_code_data
            self.order_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('order_target_server')
    server = OrderTargetServer()
    rospy.spin()
