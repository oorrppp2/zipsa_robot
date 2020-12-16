#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import numpy as np
import math

import tf2_ros


from living_lab_robot_perception.msg import QRCodeDetectAction, QRCodeDetectFeedback, QRCodeDetectResult
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from qrcode_detector_ros.msg import Result
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Empty, String, Bool, Header
from sensor_msgs.msg import JointState


class GetJointStateServer:
    def __init__(self):
        print("init")
        self.start_detect = False
        self.detect_done = False

	self.joint_sub = rospy.Subscriber('/body/joint_states', JointState, self.handle_order)
#        self.sub_order = rospy.Subscriber('get_joint_state', String, self.handle_order)
        self.get_joint_server = actionlib.SimpleActionServer('joint_state_request', QRCodeDetectAction, self.handle_request_order, False)
        # self.sub_detect = rospy.Subscriber('detected_object', Result, self.handle_detector_result)
        # self.server = actionlib.SimpleActionServer('qrcode_detect', QRCodeDetectAction, self.handle_request_detect, False)

        self.get_joint_server.start()
        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_order(self, msg):
        # print("Detector result!!!")
        if not self.start_detect:
            return

        print(msg)
        self.result_code_data = str(msg.position[-2])
        self.start_detect = False
        self.detect_done = True

        # print(self.result_orientation)

    def handle_request_order(self, goal):
        print("Request!!!")
        result = QRCodeDetectResult()
        success = True

        self.start_detect = True

        while(not self.detect_done):
            rospy.sleep(0.1)
        self.detect_done = False

        if success:
            result.result = True
            result.code_data = self.result_code_data
            self.get_joint_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('get_joint_state_server')
    server = GetJointStateServer()
    rospy.spin()
