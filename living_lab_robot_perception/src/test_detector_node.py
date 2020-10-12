#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import numpy as np
import math

import tf2_ros
import geometry_msgs


from living_lab_robot_perception.msg import QRCodeDetectAction, QRCodeDetectFeedback, QRCodeDetectResult
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from qrcode_detector_ros.msg import Result
from tf.transformations import quaternion_from_euler, quaternion_multiply


class QRCodeDetectServer:
    def __init__(self):
        print("init")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.start_detect = False
        self.detect_count = 0
        self.detect_done = False
	self.detected_pose = np.array([0.0, 0.0, 0.0])

        self.result_pose = np.array([0.0, 0.0, 0.0])
        self.result_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.result_frame_id = "base_footprint"
        self.result_code_data = ""

        self.detected_object = PointStamped()

        self.server = actionlib.SimpleActionServer('object_detect', QRCodeDetectAction, self.handle_request_detect, False)
        self.server.start()
	self.publish_count = 10
	self.publishing = False

        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_request_detect(self, goal):
        print("Received goal : " + goal.target)
	if goal.target == "":
		print("Goal target is empty")
		return
        self.goal = goal.target
        feedback = QRCodeDetectFeedback()
        result = QRCodeDetectResult()
        success = True

        result_pose = PoseStamped()

        self.result_code_data = self.result_frame_id
        rospy.sleep(2)
        if success:
            result.result = True
            result.code_data = self.result_code_data
            result.pose = result_pose
            print("Success!")
            print(result)
            self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('qrcode_detect_server')
    detect_pose_check = TF2PoseStamped()
    print("type : ",type(detect_pose_check))
    # rospy.Subscriber("/clicked_point", PointStamped, callback_point)
    server = QRCodeDetectServer()
    rospy.spin()
