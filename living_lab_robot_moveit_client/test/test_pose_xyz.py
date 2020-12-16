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

        theta = math.atan2(goal.target_pose.pose.position.y, goal.target_pose.pose.position.x)
        quat = quaternion_from_euler(0.0, 0.0, theta)
        print quat

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        joint_constraint_arm1_joint = JointConstraint()
        joint_constraint_arm1_joint.joint_name = "arm1_joint"
        joint_constraint_arm1_joint.position = float(0.0)
        joint_constraint_arm1_joint.tolerance_above = float(30) * math.pi / 180.0
        joint_constraint_arm1_joint.tolerance_below = float(30) * math.pi / 180.0
        joint_constraint_arm1_joint.weight = 1.0

        joint_constraint_arm6_joint = JointConstraint()
        joint_constraint_arm6_joint.joint_name = "arm6_joint"
        joint_constraint_arm6_joint.position = float(0.0)
        joint_constraint_arm6_joint.tolerance_above = float(10) * math.pi / 180.0
        joint_constraint_arm6_joint.tolerance_below = float(10) * math.pi / 180.0
        joint_constraint_arm6_joint.weight = 1.0
        goal.joint_constraints.append(joint_constraint_arm1_joint)
        goal.joint_constraints.append(joint_constraint_arm6_joint)

    except ValueError:
        quit()

    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    if len(sys.argv) != 5:
        print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z>"
        exit(-1)

    m = main(sys.argv[1:])
