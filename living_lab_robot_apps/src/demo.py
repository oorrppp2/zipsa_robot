#!/usr/bin/python
#-*- encoding: utf8 -*-

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

from std_msgs.msg import Empty, String
import move_base_msgs
from tf.transformations import quaternion_from_euler

from behaviors.speech import *
from behaviors.move_arm_controller import *
from behaviors.move_joint import *
from behaviors.lamp_control import *


def create_root():

    wait_start_trigger = py_trees_ros.subscribers.CheckData(name="wait_trigger", topic_name="/wait_start_trigger", topic_type=String,
        variable_name="data", expected_value="start")
    lamp_mode0 = LampControl(name="lamp_mode_0", mode=0, args="{}")
    lamp_mode1 = LampControl(name="lamp_mode_1", mode=1, args="{}")
    lamp_mode2 = LampControl(name="lamp_mode_2", mode=2, args="{}")
    lamp_mode3r = LampControl(name="lamp_mode_3r", mode=3, args="{\"color\": \"r\"}")
    lamp_mode3g = LampControl(name="lamp_mode_3g", mode=3, args="{\"color\": \"g\"}")
    lamp_mode3b = LampControl(name="lamp_mode_3b", mode=3, args="{\"color\": \"b\"}")

    turn_back = BodyRotateOnly(name="turn_back", target_pose=3.14)
    turn_front = BodyRotateOnly(name="turn_front", target_pose=0.0)


    root = py_trees.composites.Parallel("demo")
    idle = py_trees.behaviours.Running(name="done")

    #
    # Scene #1
    #
    scene1 = py_trees.composites.Sequence("scene1_intro")

    wait_scene1_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="intro")

    intro_say1 = Say(name="say_intro_question1", text='저는 무엇일까요?')
    intro_say2 = Say(name="say_intro_question2", text='에어컨? <break time="1s"/> 공기청정기? <break time="1s"/> 조명? <break time="1s"/>')
    intro_say3 = Say(name="say_hello1", text="안녕하세요? 저는 리빙랩의 새로운 로봇 '집사', 입니다.")
    intro_head_tilt_down = MoveJoint(name="tilt_down", controller_name="/head/tilt_controller", command=0.6)
    intro_say4 = Say(name="say_hello2", text='<break time="0.5s"/> 저는 머리의 프로젝터를 이용해 원격으로부터 서비스를 제공할 수 있으며, ')
    intro_head_tilt_home = MoveJoint(name="tilt_home", controller_name="/head/tilt_controller", command=0.0)

    intro_iot = py_trees.composites.Parallel('iot_intro')
    intro_say5 = Say(name="say_hello2", text="IoT를 이용해 집안 여러 가전제품을 관리할 수 있습니다.")
    intro_iot.add_children([lamp_mode2, intro_say5])

    move_arm_base1 = MoveJoint(name="move_arm_base", controller_name="/body/arm_base_controller", command=0.0)
    intro_say6 = Say(name="say_hello3", text='<break time="1.0s"/> 또, 로봇 팔을 이용해, 간단한 심부름 서비스를 제공할 수 있습니다.')
    move_arm_base2 = MoveJoint(name="move_arm_base", controller_name="/body/arm_base_controller", command=-0.15)

    scene1.add_children(
        [ wait_scene1_intro, turn_back,
          wait_start_trigger, intro_say1,
          intro_say2, turn_front, intro_say3, intro_head_tilt_down, intro_say4, intro_head_tilt_home,
          intro_iot, lamp_mode1, move_arm_base1, intro_say6, move_arm_base2 ]
    )

    #
    # Scene #2
    #

    scene2 = py_trees.composites.Sequence("scene2_living_room")

    wait_scene2_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="livingroom")

    goal_living_room = move_base_msgs.msg.MoveBaseGoal()
    goal_living_room.target_pose.header.frame_id = "map"
    goal_living_room.target_pose.header.stamp = rospy.Time.now()
    goal_living_room.target_pose.pose.position.x = 3.7
    goal_living_room.target_pose.pose.position.y = -2.56

    quat1 = quaternion_from_euler(0, 0, 2.0)
    goal_living_room.target_pose.pose.orientation.x = quat1[0]
    goal_living_room.target_pose.pose.orientation.y = quat1[1]
    goal_living_room.target_pose.pose.orientation.z = quat1[2]
    goal_living_room.target_pose.pose.orientation.w = quat1[3]

    move_to_living_room = py_trees_ros.actions.ActionClient(
        name="move to living room",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_living_room
    )

    scene2_say1 = Say(name="scene2_say1", text='<break time="1s"/> 저는 원격 프로젝션 기술을 통해, 원격 문서 공유 서비스와 원격 코칭 서비스를 지원할 수 있습니다.')
    scene2_say2 = Say(name="scene2_say2", text='원격 서비스에 대한 구체적인 데모는 죄송하지만 다음에 준비되는데로 보여드리겠습니다. <break time="1s"/> 이제 부엌을 구경해 보실까요?')

    scene2.add_children(
        [ wait_scene2_intro,
          lamp_mode3g,
          move_to_living_room,
          lamp_mode1,
          scene2_say1,
          scene2_say2]
    )

    #
    # Scene #3
    #

    scene3 = py_trees.composites.Sequence("scene3_kitchen")

    wait_scene3_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="kitchen")

    goal_kitchen = move_base_msgs.msg.MoveBaseGoal()
    goal_kitchen.target_pose.header.frame_id = "map"
    goal_kitchen.target_pose.header.stamp = rospy.Time.now()
    goal_kitchen.target_pose.pose.position.x = 4.2
    goal_kitchen.target_pose.pose.position.y = 2.2

    quat1 = quaternion_from_euler(0, 0, -2.4)
    goal_kitchen.target_pose.pose.orientation.x = quat1[0]
    goal_kitchen.target_pose.pose.orientation.y = quat1[1]
    goal_kitchen.target_pose.pose.orientation.z = quat1[2]
    goal_kitchen.target_pose.pose.orientation.w = quat1[3]

    move_to_kitchen = py_trees_ros.actions.ActionClient(
        name="move to kitchen",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_kitchen
    )

    scene3_say1 = Say(name="scene3_say1", text='<break time="1s"/> 이 스마트 식탁은, 음식을 인식해서 영양정보를 제공해줍니다. 참 편하겠죠?')

    scene3.add_children(
        [ wait_scene3_intro,
          lamp_mode3g,
          move_to_kitchen,
          lamp_mode1,
          scene3_say1]
    )

    #
    # Scene #4
    #

    scene4 = py_trees.composites.Sequence("scene4_home")

    wait_scene4_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="home_end")

    goal_home = move_base_msgs.msg.MoveBaseGoal()
    goal_home.target_pose.header.frame_id = "map"
    goal_home.target_pose.header.stamp = rospy.Time.now()
    goal_home.target_pose.pose.position.x = 0
    goal_home.target_pose.pose.position.y = -0.1

    quat1 = quaternion_from_euler(0, 0, 0)
    goal_home.target_pose.pose.orientation.x = quat1[0]
    goal_home.target_pose.pose.orientation.y = quat1[1]
    goal_home.target_pose.pose.orientation.z = quat1[2]
    goal_home.target_pose.pose.orientation.w = quat1[3]

    move_to_home = py_trees_ros.actions.ActionClient(
        name="move to home",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_home
    )

    scene4_say1 = Say(name="scene4_say1", text='<break time="1s"/> 지켜봐 주셔서 감사합니다. 이상으로 리빙랩 소개를 마치도록 하겠습니다.')
    scene4_say2 = Say(name="scene4_say2", text='소소하지만 제가 준비한 선물을 드리도록 하겠습니다. 잠시만 기다려주세요!')

    scene4.add_children(
        [ wait_scene4_intro,
          lamp_mode3g,
          move_to_home,
          lamp_mode1,
          scene4_say1,
          scene4_say2]
    )

    #
    # Scene #5
    #

    scene5 = py_trees.composites.Sequence("scene5_grasp")

    wait_scene5_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="grasp")

    goal_grasp = move_base_msgs.msg.MoveBaseGoal()
    goal_grasp.target_pose.header.frame_id = "map"
    goal_grasp.target_pose.header.stamp = rospy.Time.now()
    goal_grasp.target_pose.pose.position.x = 0.96
    goal_grasp.target_pose.pose.position.y = -1.2

    quat1 = quaternion_from_euler(0, 0, -1.5707)
    goal_grasp.target_pose.pose.orientation.x = quat1[0]
    goal_grasp.target_pose.pose.orientation.y = quat1[1]
    goal_grasp.target_pose.pose.orientation.z = quat1[2]
    goal_grasp.target_pose.pose.orientation.w = quat1[3]

    move_to_grasp = py_trees_ros.actions.ActionClient(
        name="move to grasp",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_grasp
    )

    scene5.add_children(
        [ wait_scene5_intro,
          lamp_mode3g,
          move_to_grasp,
          lamp_mode1
        ]
    )

    root.add_children([scene1, scene2, scene3, scene4, scene5, idle])


    # root.add_children([wait_for_intro_selection, turn_back, wait_trigger, say1, say2, turn_front, say3, head_tilt_down, say4, head_tilt_home, lamp_intro, lamp_mode2, move_arm_base1, say6, move_arm_base2])
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