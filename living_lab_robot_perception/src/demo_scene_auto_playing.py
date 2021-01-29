#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import numpy as np
import math

from std_msgs.msg import Empty, String, Bool, Header, Float64

"""
        scene 1 : intro. (Receive the target order.)
        scene 2 : move to table. (Zipsa moves to kichen table to find target.)
        scene 3 : find target. (Find target using yolov3 detector.)
        scene 4 : arm control. (Move arm to target object.)
        scene 5 : grasping. (Grasp the target order.)
        scene 6 : move to tea table. (Move to tea table to deliver the object to user.)
        scene 7 : put object. (Put down the object on the tea table.)
        scene 8 : move to home. (Move to home.)
"""

class DemoSceneAutoPlayer:
    def __init__(self):
        print("init")

        self.wait_scene_done = rospy.Subscriber('wait_done_scene', String, self.handle_scene)
        self.select_scene_pub = rospy.Publisher('wait_select_scene', String, queue_size=1)

        rospy.loginfo('[%s] initialized...'%rospy.get_name())

    def handle_scene(self, msg):
	print(msg)
        if msg.data == "scene_1_done":
                print("Publish : 'move_to_table'")
                self.select_scene_pub.publish(data='move_to_table')
        elif msg.data == "scene_2_done":
                print("Publish : 'find_target'")
                self.select_scene_pub.publish(data='find_target')
        elif msg.data == "scene_3_done":
                print("Publish : 'arm_control'")
                self.select_scene_pub.publish(data='arm_control')
        elif msg.data == "scene_4_done":
                print("Publish : 'grasp_object'")
                self.select_scene_pub.publish(data='grasp_object')
        elif msg.data == "scene_5_done":
                print("Publish : 'move_to_tea_table'")
                self.select_scene_pub.publish(data='move_to_tea_table')
        elif msg.data == "scene_6_done":
                print("Publish : 'put_object'")
                self.select_scene_pub.publish(data='put_object')
        elif msg.data == "scene_7_done":
                print("Publish : 'go_home'")
                self.select_scene_pub.publish(data='go_home')



if __name__ == '__main__':
    rospy.init_node('demo_scene_auto_playing')
    player = DemoSceneAutoPlayer()
    rospy.spin()
