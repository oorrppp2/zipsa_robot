#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from tf.transformations import *
from moveit_msgs.msg import JointConstraint

def main(argv):
    client = actionlib.SimpleActionClient('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction)
    client.wait_for_server()

    goal = PlanExecutePoseConstraintsGoal()
    try:
        goal.target_pose.header.frame_id = argv[0]
        goal.target_pose.pose.position.x = float(argv[1])
        goal.target_pose.pose.position.y = float(argv[2])
        goal.target_pose.pose.position.z = float(argv[3])

        roll = float(argv[4]) * math.pi / 180.0
        pitch = float(argv[5]) * math.pi / 180.0
        yaw = float(argv[6]) * math.pi / 180.0

        quat = quaternion_from_euler(roll, pitch, yaw)
        print quat

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = argv[7]
        joint_constraint.position = float(argv[8])
        joint_constraint.tolerance_above = float(argv[9]) * math.pi / 180.0
        joint_constraint.tolerance_below = float(argv[10]) * math.pi / 180.0
        joint_constraint.weight = 1.0

        goal.joint_constraints.append(joint_constraint)

    except ValueError:
        quit()

    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    if len(sys.argv) != 12:
        print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z> <r (deg)> <p (deg)> <y (deg)> <joint_name> <position> <upper_offset> <lower_offset>"
        exit(-1)

    m = main(sys.argv[1:])