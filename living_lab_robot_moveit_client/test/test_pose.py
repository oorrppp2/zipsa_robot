#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from tf.transformations import *

def main(argv):
    client = actionlib.SimpleActionClient('/plan_and_execute_pose', PlanExecutePoseAction)
    client.wait_for_server()

    goal = PlanExecutePoseGoal()
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
    except ValueError:
        quit()

    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    if len(sys.argv) != 8:
        print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z> <r (deg)> <p (deg)> <y (deg)>"
        exit(-1)

    m = main(sys.argv[1:])