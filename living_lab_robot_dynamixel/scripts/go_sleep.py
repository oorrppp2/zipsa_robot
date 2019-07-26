#!/usr/bin/env python
#-*- encoding: utf8 -*-

import sys
import rospy
import actionlib

from dynamixel_ros_control.msg import HomingAction, HomingActionGoal, HomingActionFeedback
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class DoHomingProcedure:
    def __init__(self):
        # Move arm to ready pose
        client = actionlib.SimpleActionClient('/body/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client.wait_for_server()

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = ['body_rotate_joint', 'elevation_joint', 'arm1_joint', 'arm2_joint', 'arm3_joint', 'arm4_joint', 'arm5_joint', 'arm6_joint']

        point = JointTrajectoryPoint()
        goal.trajectory.header.stamp = rospy.Time.now()
        point.positions = [0.0, -0.215, 0.0, 1.57, -3.14, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(4.0)

        client.send_goal(goal)
        client.wait_for_result()

        rospy.sleep(1.0)

        pub = rospy.Publisher('/body/arm_base_controller/command', Float64, queue_size=10)
        rospy.sleep(0.4)
        pub.publish(data=-0.15)
        rospy.sleep(2.0)

        rospy.loginfo("All homing procedure was done successfully.")
        quit()

    def func_active(self):
        rospy.loginfo("%s homing start"%self.current_target)

    def func_done(self, state, result):
        rospy.loginfo('%s homing done.'%self.current_target)


if __name__ == '__main__':
    rospy.init_node('do_homing_procedure', anonymous=False)
    m = DoHomingProcedure()
    rospy.spin()