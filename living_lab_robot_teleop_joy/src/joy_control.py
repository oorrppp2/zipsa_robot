#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import numpy as np
import math

from std_msgs.msg import Empty, String, Bool, Header, Float64
from sensor_msgs.msg import Joy
from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal

"""
        X : msg.buttons == (1,0,0,0,0,0,0,0,0,0,0,0,0)  ---->  pull out arm
        X + R2 : msg.buttons == (1,0,0,0,0,0,0,1,0,0,0,0,0)  ---->  push in arm

        O : msg.buttons == (0,1,0,0,0,0,0,0,0,0,0,0,0)  ---->  gripper close
        O + R2 : msg.buttons == (0,1,0,0,0,0,0,1,0,0,0,0,0)  ---->  gripper open

        △ : msg.buttons == (0,0,1,0,0,0,0,0,0,0,0,0,0)  ---->  grasp_ready
        △ + R2 : msg.buttons == (0,0,1,0,0,0,0,1,0,0,0,0,0)  ---->  home

        □ : msg.buttons == (0,0,0,1,0,0,0,0,0,0,0,0,0)  ---->  shake hand


        X + R1 : msg.buttons == (1,0,0,0,0,1,0,0,0,0,0,0,0)  ---->  얘들아 안녕? 나는 KIST 로봇 집사라고 해
        O + R1 : msg.buttons == (0,1,0,0,0,1,0,0,0,0,0,0,0)  ---->  얘들아 안녕?
        △ + R1 : msg.buttons == (0,0,1,0,0,1,0,0,0,0,0,0,0)  ---->  리빙랩에 놀러온걸 환영해
        □ + R1 : msg.buttons == (0,0,0,1,0,1,0,0,0,0,0,0,0)  ---->  얘들아 안녀엉 다음에 또 보자
"""

class DemoJoyControl:
    def __init__(self):
        print("init")
        self.executing = False
        self.speaking = False
        self.wait_joy_sig = rospy.Subscriber('/joy', Joy, self.handle_joy_control)

        self.pull_out_arm = rospy.Publisher('/body/arm_base_controller/command', Float64, queue_size=1)
        self.pull_out_arm_state = False

        self.gripper = rospy.Publisher('/body/gripper_controller/command', Float64, queue_size=1)
        self.gripper_close = False

        self.tts_request = rospy.Publisher('/tts_request', String, queue_size=1)

        self.client = actionlib.SimpleActionClient('/plan_and_execute_named_pose', PlanExecuteNamedPoseAction)
        self.client.wait_for_server()


        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_joy_control(self, msg):
        print(msg.buttons)
        if msg.buttons == (1,0,0,0,0,0,0,0,0,0,0,0,0):
            if self.pull_out_arm_state == False:
                self.pull_out_arm.publish(0)
                self.pull_out_arm_state = True
        elif msg.buttons == (1,0,0,0,0,0,0,1,0,0,0,0,0):
            if self.pull_out_arm_state == True:
                self.pull_out_arm.publish(-0.15)
                self.pull_out_arm_state = False
        elif msg.buttons == (0,1,0,0,0,0,0,0,0,0,0,0,0):
            if self.gripper_close == False:
                self.gripper.publish(0.0)
                self.gripper_close = True
        elif msg.buttons == (0,1,0,0,0,0,0,1,0,0,0,0,0):
            if self.gripper_close == True:
                self.gripper.publish(1.0)
                self.gripper_close = False
        elif msg.buttons == (0,0,1,0,0,0,0,0,0,0,0,0,0):
            if self.executing == False:
                self.executing = True
                goal = PlanExecuteNamedPoseGoal()
                goal.target_name = "grasp_ready"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                print(self.client.get_result())
        elif msg.buttons == (0,0,1,0,0,0,0,1,0,0,0,0,0):
            if self.executing == False:
                self.executing = True
                goal = PlanExecuteNamedPoseGoal()
                goal.target_name = "home"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                print(self.client.get_result())
        elif msg.buttons == (1,0,0,0,0,1,0,0,0,0,0,0,0):
            if self.speaking == False:
                self.speaking = True
                self.tts_request.publish("얘들아 안녕? 나는 KIST 로봇 집사라고 해")
        elif msg.buttons == (0,1,0,0,0,1,0,0,0,0,0,0,0):
            if self.speaking == False:
                self.speaking = True
                self.tts_request.publish("얘들아 안녕?")
        elif msg.buttons == (0,0,1,0,0,1,0,0,0,0,0,0,0):
            if self.speaking == False:
                self.speaking = True
                self.tts_request.publish("리빙랩에 놀러온걸 환영해")
        elif msg.buttons == (0,0,0,1,0,1,0,0,0,0,0,0,0):
            if self.speaking == False:
                self.speaking = True
                self.tts_request.publish("얘들아 안녀엉 다음에 또 보자")
        elif msg.buttons == (0,0,0,1,0,0,0,0,0,0,0,0,0):
            if self.executing == False:
                self.executing = True
                goal = PlanExecuteNamedPoseGoal()
                goal.target_name = "shake_arm_left"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                goal.target_name = "shake_arm_right"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                goal.target_name = "shake_arm_left"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                goal.target_name = "shake_arm_right"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                goal.target_name = "shake_arm_left"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                goal.target_name = "shake_arm_right"
                self.client.send_goal(goal)
                self.client.wait_for_result()
                # print(self.client.get_result())
        elif msg.buttons == (0,0,0,0,0,0,0,0,0,0,0,0,0):
            self.executing = False
            self.speaking = False



if __name__ == '__main__':
    rospy.init_node('joy_control')
    player = DemoJoyControl()
    rospy.spin()
