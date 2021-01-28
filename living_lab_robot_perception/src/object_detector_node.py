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


class ObjectDetectServer:
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
        self.result_data = ""

        self.detected_object = PointStamped()

        self.sub_detect = rospy.Subscriber('detected_object', Result, self.handle_detector_result)
        self.server = actionlib.SimpleActionServer('object_detect', ObjectDetectAction, self.handle_request_detect, False)
        self.server.start()
	self.publish_count = 10
	self.publishing = False
        self.clear_target_region = False

        self.clear_buffer_count = 5    # 5개 메시지 무시

        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_detector_result(self, msg):
        #print("Detector result!!!")
	#print(msg)
        if self.start_detect:
                if self.clear_buffer_count > 0:
                        print("Clearing buffer ...")
                        self.clear_buffer_count -= 1
                        return
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "camera_color_optical_frame"
	t.child_frame_id = "object_coordinate"
	t.transform.rotation.x = 0
	t.transform.rotation.y = 0
	t.transform.rotation.z = 0
	t.transform.rotation.w = 1
  	t.transform.translation.x = self.detected_pose[0]
  	t.transform.translation.y = self.detected_pose[1]
	t.transform.translation.z = self.detected_pose[2]

	if not self.publishing:
		if not self.start_detect:
			br.sendTransform(t)
			return
		if msg.data != self.goal:
			br.sendTransform(t)
			return

		print("Detection succeed!!")
		print("Result with (camera coordinate)")
		print(msg)
		
	        self.result_frame_id = msg.data
		self.detected_pose[0] = msg.pose.pose.position.x
		self.detected_pose[1] = msg.pose.pose.position.y
		self.detected_pose[2] = msg.pose.pose.position.z
		self.publishing = True
	else:
		br.sendTransform(t)

		# Publish at least 10 times.
		if self.publish_count > 0:
			print("Publishing")
			self.publish_count = self.publish_count - 1
#			rospy.sleep(1)
			return
		else:
			print("Publish done")
			self.publish_count = 10
			self.publishing = False
		        self.detect_done = True

        self.start_detect = False
        self.clear_buffer_count = 5

#        self.detected_object.point.x = msg.pose.pose.position.x
#        self.detected_object.point.y = msg.pose.pose.position.y
#        self.detected_object.point.z = msg.pose.pose.position.z
        self.detected_object.point.x = 0
        self.detected_object.point.y = 0
        self.detected_object.point.z = 0
        self.result_orientation = np.array(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])


        # print(self.result_orientation)

    def handle_request_detect(self, goal):
        print("Received goal : " + goal.target)
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
        target_detect_pose = self.tf_buffer.transform(detect_pose, "base_footprint")
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
        if success:
                result.result = True
                result.data = self.result_data
                result.pose = result_pose
                print(result)
                self.server.set_succeeded(result)
                self.clear_target_region = True

        if self.clear_target_region:
                # Obstacle add into detected object region.
                print("Obstacle add into detected object region.")
                pub = rospy.Publisher("/add_obstacle", String, queue_size=1)
                rospy.sleep(0.4)
                pub.publish(data=str(result_pose.pose.position.x) + ' ' + \
                                        str(result_pose.pose.position.y) + ' ' + \
                                        str(result_pose.pose.position.z) + ' ' + \
                                        str(self.result_data) + ' ' + ' 0.5 0.5 0.5')
                rospy.sleep(2.0)
                # Remove target region points to clearing.
                print("Remove target region points to clearing.")
                pub = rospy.Publisher("/remove_points_request", String, queue_size=1)
                rospy.sleep(0.4)
                pub.publish(data=self.result_data)
                rospy.sleep(2.0)
                # Remove added obstacle to clear the target region.
                print("Remove added obstacle to clear the target region.")
                pub = rospy.Publisher("/del_all_obstacles", String, queue_size=1)
                rospy.sleep(0.4)
                pub.publish(data='1')
                # rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('object_detect_server')
    detect_pose_check = TF2PoseStamped()
    print("type : ",type(detect_pose_check))
    # rospy.Subscriber("/clicked_point", PointStamped, callback_point)
    server = ObjectDetectServer()
    rospy.spin()
