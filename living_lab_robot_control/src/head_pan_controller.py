#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import rospy
import tf2_ros
import threading
import math

from std_msgs.msg import Float64
from tf2_geometry_msgs import PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class HeadPanController:
    def __init__(self):
        self.lock = threading.RLock()
        self.tf_buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buf)
        self.prev_head1_pan = 0.0
        self.prev_head2_pan = 0.0

        self.pub_head1_pan = rospy.Publisher('head1_pan_controller/command', Float64, queue_size=10)
        self.pub_head2_pan = rospy.Publisher('head2_pan_controller/command', Float64, queue_size=10)
        self.pub_head2_d435_tilt = rospy.Publisher('head2_d435_tilt_controller/command', Float64, queue_size=10)

        with self.lock:
            self.head1_target_point = PointStamped()
            self.head1_target_point.header.frame_id = "base_footprint"
            self.head1_target_point.point.x = 2.0
            self.head1_target_point.point.y = 0.0
            self.head1_target_point.point.z = 0.0

            self.head2_target_point = PointStamped()
            self.head2_target_point.header.frame_id = "base_footprint"
            self.head2_target_point.point.x = 1.5
            self.head2_target_point.point.y = 0.0
            self.head2_target_point.point.z = 0.0

        rospy.Timer(rospy.Duration(0.05), self.handle_head_controller)
        rospy.loginfo('[%s] initialzed...' % rospy.get_name())

    def handle_head_controller(self, event):
        with self.lock:
            try:
                self.head1_target_point.header.stamp = rospy.Time()
                self.head2_target_point.header.stamp = rospy.Time()
                point_transformed1 = self.tf_buf.transform(self.head1_target_point, 'body_assy')
                point_transformed2 = self.tf_buf.transform(self.head2_target_point, 'body_assy')
                point_transformed3 = self.tf_buf.transform(self.head2_target_point, 'gaze_link')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("warn for lookup transform.")
                return

        pan_angle1 = math.atan2(point_transformed1.point.y, point_transformed1.point.x)
        pan_angle2 = math.atan2(point_transformed2.point.y, point_transformed2.point.x)

        tilt_angle = math.atan2(point_transformed3.point.z, point_transformed3.point.x)

        if math.copysign(1, pan_angle1) != math.copysign(1, self.prev_head1_pan):
            if math.fabs(pan_angle1) > math.pi/2:
                pan_angle1 = pan_angle1 + (math.copysign(1, self.prev_head1_pan) * math.pi * 2)

        if math.copysign(1, pan_angle2) != math.copysign(1, self.prev_head2_pan):
            if math.fabs(pan_angle2) > math.pi/2:
                pan_angle2 = pan_angle2 + (math.copysign(1, self.prev_head2_pan) * math.pi * 2)

        self.pub_head1_pan.publish(pan_angle1)
        self.pub_head2_pan.publish(pan_angle2)
        self.pub_head2_d435_tilt.publish(-1 * tilt_angle)

        self.prev_head1_pan = pan_angle1
        self.prev_head2_pan = pan_angle2


if __name__ == '__main__':
    rospy.init_node('head_pan_controller', anonymous=False)
    try:
        m = HeadPanController()
        rospy.spin()
    except rospy.ROSInterruptException: pass