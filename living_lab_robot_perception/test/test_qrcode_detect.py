#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib

from living_lab_robot_perception.msg import QRCodeDetectAction, QRCodeDetectGoal


def main():
    client = actionlib.SimpleActionClient('qrcode_detect', QRCodeDetectAction)
    client.wait_for_server()

    goal = QRCodeDetectGoal()
    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()


if __name__ == '__main__':
    rospy.init_node('test_qrcode', anonymous=False)
    m = main()