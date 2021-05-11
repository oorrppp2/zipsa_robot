#!/usr/bin/python
#-*- encoding: utf8 -*-

import sys
import rospy
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal

def main(argv):
    client = actionlib.SimpleActionClient('/plan_and_execute_named_pose', PlanExecuteNamedPoseAction)
    client.wait_for_server()

    goal = PlanExecuteNamedPoseGoal()
    goal.target_name = argv

    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()


if __name__ == '__main__':
    rospy.init_node('test_named_pose', anonymous=False)

    if len(sys.argv) != 2:
        print "Usage: rosrun living_lab_robot_moveit_client test_named_pose <named_pose>"
        exit(-1)

    m = main(sys.argv[1])
