#!/usr/bin/python
#-*- encoding: utf8 -*-

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

from behaviors.speech import *
from behaviors.move_arm_controller import *
from behaviors.move_joint import *
from behaviors.lamp_control import *
from std_msgs.msg import Empty, String

def create_root():
    root = py_trees.composites.Sequence("demo_intro_myself")

    wait_for_intro_selection = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="intro")

    turn_back = BodyRotateOnly(name="turn_back", target_pose=3.14)
    wait_trigger = py_trees_ros.subscribers.CheckData(name="wait_trigger", topic_name="/wait_start_trigger", topic_type=String,
        variable_name="data", expected_value="start")

    say1 = Say(name="say_intro_question1", text='저는 무엇일까요?')
    say2 = Say(name="say_intro_question2", text='에어컨? <break time="1s"/> 공기청정기? <break time="1s"/> 조명? <break time="1s"/>')
    turn_front = BodyRotateOnly(name="body_rotate1", target_pose=0.0)
    say3 = Say(name="say_hello1", text="안녕하세요? 저는 리빙랩의 새로운 로봇 '집사', 입니다.")
    head_tilt_down = MoveJoint(name="tilt_down", controller_name="/head/tilt_controller", command=0.6)
    say4 = Say(name="say_hello2", text='<break time="0.5s"/> 저는 머리의 프로젝터를 이용해 원격으로부터 서비스를 제공할 수 있으며, ')
    head_tilt_home = MoveJoint(name="tilt_home", controller_name="/head/tilt_controller", command=0.0)

    lamp_intro = py_trees.composites.Parallel('lamp_intro')
    lamp_mode1 = LampControl(name="lamp_mode_iot", mode=2, args="{}")
    say5 = Say(name="say_hello2", text="IoT를 이용해 집안 여러 가전제품을 관리할 수 있습니다.")

    lamp_intro.add_children([lamp_mode1, say5])

    lamp_mode2 = LampControl(name="lamp_mode_normal", mode=1, args="{}")
    move_arm_base1 = MoveJoint(name="move_arm_base", controller_name="/body/arm_base_controller", command=0.0)
    say6 = Say(name="say_hello3", text='<break time="1.0s"/> 또, 로봇 팔을 이용해, 간단한 심부름 서비스를 제공할 수 있습니다.')
    move_arm_base2 = MoveJoint(name="move_arm_base", controller_name="/body/arm_base_controller", command=-0.15)

    idle = py_trees.behaviours.Running(name="done")

    root.add_children([wait_for_intro_selection, turn_back, wait_trigger, say1, say2, turn_front, say3, head_tilt_down, say4, head_tilt_home, lamp_intro, lamp_mode2, move_arm_base1, say6, move_arm_base2, idle])
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

if __name__ == '__main__':
    rospy.init_node('app_introduction', anonymous=False)
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(500)