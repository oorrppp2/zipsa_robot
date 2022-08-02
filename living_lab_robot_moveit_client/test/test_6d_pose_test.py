#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
import tf
from tf.transformations import *
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
import geometry_msgs.msg
import numpy as np
# from geometry_msgs.msg import TransformStamped
import tf2_ros

def main(argv):
    client = actionlib.SimpleActionClient('/plan_and_execute_pose', PlanExecutePoseAction)
    client.wait_for_server()

    goal = PlanExecutePoseGoal()
    sub_goal = PlanExecutePoseGoal()

    # try:
    # position:
    #   x: 0.794200725615
    #   y: 0.0603146463652
    #   z: 0.600947724395
    # orientation:
    #   x: 0.22358160462438068
    #   y: 0.30621366750345297
    #   z: -0.5968402140862933
    #   w: 0.7071253175760994

    #   x: -0.5178851437979983
    #   y: 0.4730788599043741
    #   z: 0.5028019469428547
    #   w: 0.5051549982880292

#   x = -0.028853754202135252
#   y = 0.004224032486704575
#   z = -0.6914959052225815
#   w = 0.7217915429529475
    #   x: -0.24064532415948148
    #   y: -0.289347082505064
    #   z: -0.6399098241797725
    #   w: 0.6699876944572927
    #   x: -0.2762711990653907
    #   y: 0.234196888530125
    #   z: 0.6668545749245718
    #   w: 0.6512534206220709
    #   x: 0.6296955187676937
    #   y: -0.05200468500251687
    #   z: 0.785308724473028
    # orientation:
    #   x: 0.25194559919970655
    #   y: 0.2727012140252847
    #   z: -0.5930299651383254
    #   w: 0.7144738787115126


    goal.target_pose.header.frame_id = "base_footprint"
    # goal.target_pose.pose.position.x = 0.594200725615
    # goal.target_pose.pose.position.y = 0.0603146463652
    # goal.target_pose.pose.position.z = 0.700947724395

    # goal.target_pose.pose.orientation.x = -0.2762711990653907
    # goal.target_pose.pose.orientation.y = 0.234196888530125
    # goal.target_pose.pose.orientation.z = 0.6668545749245718
    # goal.target_pose.pose.orientation.w = 0.6512534206220709

    goal.target_pose.pose.position.x = 0.67413
    goal.target_pose.pose.position.y = -0.1044
    goal.target_pose.pose.position.z = 0.77486

    goal.target_pose.pose.orientation.x = -0.21786
    goal.target_pose.pose.orientation.y = 0.30333
    goal.target_pose.pose.orientation.z = 0.52865
    goal.target_pose.pose.orientation.w = 0.76226

    trans = np.asarray([goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z])
    quat = np.asarray([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])
    # roll, pitch, yaw = euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
    # goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])

    R = quaternion_matrix(quat)[:3,:3]
    XL2 = np.matmul(R.T, trans)
    XL2[0] += 0.1

    sub_trans = np.matmul(R, XL2)

    print(trans)
    print(sub_trans)


    sub_goal.target_pose.header.frame_id = "base_footprint"
    sub_goal.target_pose.pose.position.x = sub_trans[0]
    sub_goal.target_pose.pose.position.y = sub_trans[1]
    sub_goal.target_pose.pose.position.z = sub_trans[2]

    sub_goal.target_pose.pose.orientation.x = quat[0]
    sub_goal.target_pose.pose.orientation.y = quat[1]
    sub_goal.target_pose.pose.orientation.z = quat[2]
    sub_goal.target_pose.pose.orientation.w = quat[3]

    # except ValueError:
    #     quit()

    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()

    if client.get_result() == True:

        client.send_goal(sub_goal)
        client.wait_for_result()

        print client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    # if len(sys.argv) != 8:
    #     print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z> <r (deg)> <p (deg)> <y (deg)>"
    #     exit(-1)

    m = main(sys.argv[1:])

    # rospy.spin()