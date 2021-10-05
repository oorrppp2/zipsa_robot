#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from tf.transformations import *
from moveit_msgs.msg import JointConstraint

# import tf2_ros
# from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

import tf

def main(argv):
    # tf_buffer = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buffer)
    # target_detect_pose = tf_buffer.lookup_transform("end_effector", "base_footprint", rospy.Time())
    listener = tf.TransformListener()
    listener.waitForTransform('/base_footprint','/end_effector',rospy.Time(), rospy.Duration(4.0))
    (trans, quat) = listener.lookupTransform('/base_footprint', '/end_effector', rospy.Time(0))

    print(trans)
    print(quat)

    client = actionlib.SimpleActionClient('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction)
    client.wait_for_server()

    goal = PlanExecutePoseConstraintsGoal()

    # exit(0)
    try:
        goal.target_pose.header.frame_id = argv[0]
        goal.target_pose.pose.position.x = trans[0]
        goal.target_pose.pose.position.y = trans[1]
        goal.target_pose.pose.position.z = trans[2] - 0.13

        # theta = math.atan2(goal.target_pose.pose.position.y, goal.target_pose.pose.position.x)
        # quat = quaternion_from_euler(0.0, 0.0, theta)
        # print quat

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        joint_constraint_body_rotate_joint = JointConstraint()
        joint_constraint_body_rotate_joint.joint_name = "body_rotate_joint"
        joint_constraint_body_rotate_joint.position = float(0.0)
        joint_constraint_body_rotate_joint.tolerance_above = float(5) * math.pi / 180.0
        joint_constraint_body_rotate_joint.tolerance_below = float(5) * math.pi / 180.0
        joint_constraint_body_rotate_joint.weight = 1.0

        # joint_constraint_arm4_joint = JointConstraint()
        # joint_constraint_arm4_joint.joint_name = "arm4_joint"
        # joint_constraint_arm4_joint.position = float(0.0)
        # joint_constraint_arm4_joint.tolerance_above = float(10) * math.pi / 180.0
        # joint_constraint_arm4_joint.tolerance_below = float(10) * math.pi / 180.0
        # joint_constraint_arm4_joint.weight = 1.0

        # joint_constraint_arm6_joint = JointConstraint()
        # joint_constraint_arm6_joint.joint_name = "arm6_joint"
        # joint_constraint_arm6_joint.position = float(0.0)
        # joint_constraint_arm6_joint.tolerance_above = float(10) * math.pi / 180.0
        # joint_constraint_arm6_joint.tolerance_below = float(10) * math.pi / 180.0
        # joint_constraint_arm6_joint.weight = 1.0
        goal.joint_constraints.append(joint_constraint_body_rotate_joint)
        # goal.joint_constraints.append(joint_constraint_arm6_joint)

    except ValueError:
        quit()

    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    # if len(sys.argv) != 5:
    #     print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z>"
    #     exit(-1)

    m = main(sys.argv[1:])
