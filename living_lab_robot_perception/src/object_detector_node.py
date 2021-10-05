#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import numpy as np
import math

import tf2_ros
import geometry_msgs
from std_msgs.msg import Empty, String, Bool, Header, Float64


from living_lab_robot_perception.msg import ObjectDetectAction, ObjectDetectFeedback, ObjectDetectResult
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from living_lab_robot_perception.msg import Result
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_srvs.srv import Empty



class ObjectDetectServer:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.start_detect = False
        self.detect_done = False

        self.result_pose = np.array([0.0, 0.0, 0.0])
        self.result_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.result_frame_id = "base_footprint"
        self.result_data = ""

        self.detected_object = PointStamped()

        self.remove_pub = rospy.Publisher("/remove_points_request", String, queue_size=1)
        self.request_target_pub = rospy.Publisher("/request_target", String, queue_size=1)
        self.sub_detect = rospy.Subscriber('detected_object', Result, self.handle_detector_result)
        self.server = actionlib.SimpleActionServer('object_detect', ObjectDetectAction, self.handle_request_detect, False)
        self.server.start()
        self.clear_target_region = False

        self.clear_buffer_count = 5    # 5개 메시지 무시
        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_detector_result(self, msg):
        if self.start_detect:
                if self.clear_buffer_count > 0:
                        print("Clearing buffer ...")
                        self.clear_buffer_count -= 1
                        return
                else:
                    self.start_detect = False
                    self.clear_buffer_count = 5
                    self.result_frame_id = msg.data
                    self.detected_object.point.x = 0
                    self.detected_object.point.y = 0
                    self.detected_object.point.z = 0
                    self.result_orientation = np.array(
                        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])

                    self.detect_done = True

    def handle_request_detect(self, goal):
        print("Received goal : " + goal.target)
        success = False
        self.request_target_pub.publish(data=goal.target)
        self.detected_pose = np.array([0.0, 0.0, 0.0])
        if goal.target == "":
                print("Goal target is empty")
                return
        self.goal = goal.target
        feedback = ObjectDetectFeedback()
        result = ObjectDetectResult()
        success = True

        self.result_pose = np.array([0.0, 0.0, 0.0])
        self.start_detect = True

        while(not self.detect_done):
            rospy.sleep(0.1)
        self.detect_done = False

        # print("detect_done : ", self.detect_done)

        detect_pose = TF2PoseStamped()
        #print("type : ", detect_pose)
        detect_pose.header.frame_id = "object_coordinate"
#        detect_pose.header.stamp = rospy.Time.now()
        detect_pose.pose.position.x = self.detected_object.point.x
        detect_pose.pose.position.y = self.detected_object.point.y
        detect_pose.pose.position.z = self.detected_object.point.z

        quat1 = [self.result_orientation[0], self.result_orientation[1], self.result_orientation[2], self.result_orientation[3]]
        quat1 = quaternion_multiply(quaternion_from_euler(math.pi/2, 0, 0), quat1)
        quat_result = quaternion_multiply(quaternion_from_euler(0, -math.pi/2, 0), quat1)
        detect_pose.pose.orientation.x = quat_result[0]
        detect_pose.pose.orientation.y = quat_result[1]
        detect_pose.pose.orientation.z = quat_result[2]
        detect_pose.pose.orientation.w = quat_result[3]

        # convert detect_pose with reference base_footprint
        target_detect_pose = self.tf_buffer.transform(detect_pose, "base_footprint", timeout=rospy.Duration(10.0))

        # target_detect_pose.pose.position.z -= 0.090

        result_pose = PoseStamped()
        # result_pose.header.frame_id = self.result_frame_id
        result_pose.pose.position.x = target_detect_pose.pose.position.x
        result_pose.pose.position.y = target_detect_pose.pose.position.y
        result_pose.pose.position.z = target_detect_pose.pose.position.z
        result_pose.pose.orientation.x = target_detect_pose.pose.orientation.x
        result_pose.pose.orientation.y = target_detect_pose.pose.orientation.y
        result_pose.pose.orientation.z = target_detect_pose.pose.orientation.z
        result_pose.pose.orientation.w = target_detect_pose.pose.orientation.w

        self.result_data = self.result_frame_id

        # Remove target region points to clearing.
        print("Remove target region points to clearing.")
        self.remove_pub.publish(data="remove")
        # rospy.sleep(0.4)
        rospy.sleep(2.0)
        # Obstacle add into detected object region.
        print("Clearing octomap...")
        self.clear_octomap()

        if success:
                result.result = True
                result.data = self.result_data
                result.pose = result_pose
                print(result)
                self.server.set_succeeded(result)
                self.clear_target_region = True

if __name__ == '__main__':
    rospy.init_node('object_detect_server')
    server = ObjectDetectServer()
    rospy.spin()
