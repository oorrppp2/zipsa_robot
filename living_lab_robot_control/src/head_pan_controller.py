#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import rospy
import tf2_ros
import threading
import math

from std_msgs.msg import Float64, Bool
from tf2_geometry_msgs import PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class HeadPanController:
    def __init__(self):
        self.lock = threading.RLock()
        self.tf_buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buf)
        self.prev_head_pan = 0.0
        self.enable_gaze_sync = True

        self.result_pan_angle = 0.0
        self.result_tilt_angle = 0.0

        self.sub_gaze_sync = rospy.Subscriber('set_gaze_sync', Bool, self.handle_set_gaze_sync)
        self.sub_gaze_target = rospy.Subscriber('set_gaze_target', PointStamped, self.handle_set_gaze_target)

        self.pub_head_pan = rospy.Publisher('/head/pan_controller/command', Float64, queue_size=10)
        self.pub_head_tilt = rospy.Publisher('/head/tilt_controller/command', Float64, queue_size=10)

        with self.lock:
            self.head_target_point = PointStamped()
            self.head_target_point.header.frame_id = "base_footprint"
            self.head_target_point.point.x = 2.0
            self.head_target_point.point.y = 0.0
            self.head_target_point.point.z = 1.291

        rospy.Timer(rospy.Duration(0.02), self.handle_head_controller)
        rospy.loginfo('[%s] initialzed...' % rospy.get_name())

    def handle_set_gaze_sync(self, msg):
        with self.lock:
            self.enable_gaze_sync = msg.data

            if msg.data:
                self.head_target_point = PointStamped()
                self.head_target_point.header.frame_id = "base_footprint"
                self.head_target_point.point.x = 2.0
                self.head_target_point.point.y = 0.0
                self.head_target_point.point.z = 1.291

    def handle_set_gaze_target(self, msg):
        with self.lock:
            self.head_target_point = msg

            try:
                self.head_target_point.header.stamp = rospy.Time()
                point_transformed1 = self.tf_buf.transform(self.head_target_point, 'body_assy')
                point_transformed2 = self.tf_buf.transform(self.head_target_point, 'gaze_link')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("warn for lookup transform.")
                return

            self.result_pan_angle = math.atan2(point_transformed1.point.y, point_transformed1.point.x)
            self.result_tilt_angle = -1.0 *  math.atan2(point_transformed2.point.z, point_transformed2.point.x)
            if self.result_tilt_angle < 0:
                self.result_tilt_angle = 0.0
            elif self.result_tilt_angle > 0.6:
                self.result_tilt_angle = 0.6

    def handle_head_controller(self, event):
        with self.lock:
            try:
                self.head_target_point.header.stamp = rospy.Time()
                point_transformed1 = self.tf_buf.transform(self.head_target_point, 'body_assy')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("warn for lookup transform.")
                return
            self.result_pan_angle = math.atan2(point_transformed1.point.y, point_transformed1.point.x)

        if math.copysign(1, self.result_pan_angle) != math.copysign(1, self.prev_head_pan):
            if math.fabs(self.result_pan_angle) > math.pi/2:
                self.result_pan_angle = self.result_pan_angle + (math.copysign(1, self.prev_head_pan) * math.pi * 2)

        if not self.enable_gaze_sync:
            self.pub_head_pan.publish(self.result_pan_angle)
            self.pub_head_tilt.publish(self.result_tilt_angle)
        else:
            self.pub_head_pan.publish(0.0)
        self.prev_head_pan = self.result_pan_angle


if __name__ == '__main__':
    rospy.init_node('head_pan_controller', anonymous=False)
    try:
        m = HeadPanController()
        rospy.spin()
    except rospy.ROSInterruptException: pass