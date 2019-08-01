#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import numpy as np
import math

import tf2_ros


from living_lab_robot_perception.msg import QRCodeDetectAction, QRCodeDetectFeedback, QRCodeDetectResult
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from qrcode_detector_ros.msg import Result
from tf.transformations import quaternion_from_euler, quaternion_multiply


class QRCodeDetectServer:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.start_detect = False
        self.detect_count = 0
        self.detect_done = False

        self.result_pose = np.array([0.0, 0.0, 0.0])
        self.result_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.result_frame_id = "base_footprint"
        self.result_code_data = ""

        self.sub_detect = rospy.Subscriber('detected_code', Result, self.handle_detector_result)
        self.server = actionlib.SimpleActionServer('qrcode_detect', QRCodeDetectAction, self.handle_request_detect, False)
        self.server.start()
        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_detector_result(self, msg):
        if not self.start_detect:
            return

        if self.detect_count < 10:
            self.result_pose += np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            self.detect_count += 1
        else:
            self.result_pose = self.result_pose / 10.0
            self.result_orientation = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            self.result_frame_id = msg.pose.header.frame_id
            self.result_code_data = msg.data
            self.detect_done = True
            self.start_detect = False
            self.detect_count = 0

    def handle_request_detect(self, goal):
        feedback = QRCodeDetectFeedback()
        result = QRCodeDetectResult()
        success = True

        self.result_pose = np.array([0.0, 0.0, 0.0])
        self.start_detect = True

        while(not self.detect_done):
            rospy.sleep(0.1)
        self.detect_done = False

        detect_pose = TF2PoseStamped()
        detect_pose.header.frame_id = self.result_frame_id
        #detect_pose.header.stamp = rospy.Time.now()
        detect_pose.pose.position.x = self.result_pose[0]
        detect_pose.pose.position.y = self.result_pose[1]
        detect_pose.pose.position.z = self.result_pose[2]

        quat1 = [self.result_orientation[0], self.result_orientation[1], self.result_orientation[2], self.result_orientation[3]]
        quat1 = quaternion_multiply(quaternion_from_euler(math.pi/2, 0, 0), quat1)
        quat_result = quaternion_multiply(quaternion_from_euler(0, -math.pi/2, 0), quat1)
        detect_pose.pose.orientation.x = quat_result[0]
        detect_pose.pose.orientation.y = quat_result[1]
        detect_pose.pose.orientation.z = quat_result[2]
        detect_pose.pose.orientation.w = quat_result[3]

        # convert detect_pose with reference base_footprint
        target_detect_pose = self.tf_buffer.transform(detect_pose, "base_footprint")
        target_detect_pose.pose.position.z -= 0.090

        result_pose = PoseStamped()
        result_pose.pose.position.x = target_detect_pose.pose.position.x
        result_pose.pose.position.y = target_detect_pose.pose.position.y
        result_pose.pose.position.z = target_detect_pose.pose.position.z
        result_pose.pose.orientation.x = target_detect_pose.pose.orientation.x
        result_pose.pose.orientation.y = target_detect_pose.pose.orientation.y
        result_pose.pose.orientation.z = target_detect_pose.pose.orientation.z
        result_pose.pose.orientation.w = target_detect_pose.pose.orientation.w

        if success:
            result.result = True
            result.code_data = self.result_code_data
            result.pose = result_pose
            self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('qrcode_detect_server')
    server = QRCodeDetectServer()
    rospy.spin()