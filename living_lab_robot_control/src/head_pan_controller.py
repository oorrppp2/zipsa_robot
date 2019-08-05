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
        self.enable_gaze_sync = False

        #self.sub_gaze_sync = rospy.Subscriber('set_gaze_sync', Bool, self.handle_set_gaze_sync)
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

    def handle_set_gaze_target(self, msg):
        with self.lock:
            self.head_target_point = msg

    def handle_head_controller(self, event):
        with self.lock:
            try:
                self.head_target_point.header.stamp = rospy.Time()
                point_transformed1 = self.tf_buf.transform(self.head_target_point, 'body_assy')
                point_transformed2 = self.tf_buf.transform(self.head_target_point, 'gaze_link')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("warn for lookup transform.")
                return

        pan_angle1 = math.atan2(point_transformed1.point.y, point_transformed1.point.x)
        tilt_angle = math.atan2(point_transformed2.point.z, point_transformed1.point.x)

        if math.copysign(1, pan_angle1) != math.copysign(1, self.prev_head_pan):
            if math.fabs(pan_angle1) > math.pi/2:
                pan_angle1 = pan_angle1 + (math.copysign(1, self.prev_head_pan) * math.pi * 2)

        tilt_angle = -1 * tilt_angle
        if tilt_angle < 0:
            tilt_angle = 0.0
        elif tilt_angle > 0.6:
            tilt_angle = 0.6

        self.pub_head_pan.publish(pan_angle1)
        #self.pub_head_tilt.publish(tilt_angle)
        rospy.loginfo("%f %f"%(pan_angle1, tilt_angle))

        self.prev_head_pan = pan_angle1


if __name__ == '__main__':
    rospy.init_node('head_pan_controller', anonymous=False)
    try:
        m = HeadPanController()
        rospy.spin()
    except rospy.ROSInterruptException: pass