#!/usr/bin/python
#-*- encoding: utf8 -*-

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import tf2_ros

from std_msgs.msg import Empty, String, Bool, Header
from geometry_msgs.msg import PointStamped
import move_base_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

from behaviors.speech import *
from behaviors.move_arm_controller import *
from behaviors.move_joint import *
from behaviors.lamp_control import *
from behaviors.wait_time import *
from behaviors.actions import *
from behaviors.gaze_sync_control import *
from behaviors.app_control import *
from behaviors.utils import *

from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from living_lab_robot_perception.msg import ObjectDetectAction, ObjectDetectGoal
from living_lab_robot_perception.msg import ReceiveTargetAction, ReceiveTargetGoal

global Point_data
global Point_flag

def create_root():

    root = py_trees.composites.Parallel("demo")
    done_scene = DonePlayScene(name="done_scene")

    intro = py_trees.composites.Sequence("intro")

    wait_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="intro")

#    start_scene1 = Print_message(name="* Move to ready pose *")
    start_mention1 = Print_message(name="* Introduce *")

    scene1_say1 = Say(name="say_hello1", text='안녕하세요? 저는 한국과학기술연구원에서 개발한 리빙랩의 로봇, 집사입니다.')
    """
        적절한 멘트
    """
    done_scene_1 = Publish(topic_name="/wait_done_scene", data="scene_1_done")
    intro.add_children(
        [wait_intro,
         start_mention1,
         scene1_say1,
         done_scene_1,
         ]
    )

    #
    # Scene 3 Order the target
    #
    order_the_target = py_trees.composites.Sequence("order_the_target")

    wait_order_the_target = py_trees_ros.subscribers.CheckData(name="wait_order_the_target", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="order_the_target")

    order_mention1 = Print_message(name="Choose cup / bottle / milk  or go home?")
    scene1_say2 = Say(name="say_request1", text='어떤 물건을 가져다 드릴까요?')
    # order_target_action = OrderActionClient(
    #     name="order_target",
    #     action_namespace="/order_received",
    #     action_spec=ReceiveTargetAction,
    #     action_goal=ReceiveTargetGoal()
    # )
    order_object = OrderActionClient(
        name="order_received",
        action_namespace="/sst_order_received",
        action_spec=ReceiveTargetAction,
        action_goal=ReceiveTargetGoal()
    )

    order_the_target.add_children(
        [wait_order_the_target,
         order_mention1,
         scene1_say2,
#         order_target_action,
         order_object,
         ]
    )

#    root.add_children([gripper_open_cmd, intro, order_the_target, move_to_user, find_target, arm_control, move_to_tea_table, grasp_object, go_home, finish_demo, put_object, elevation_up, elevation_down])
    root.add_children([intro, order_the_target])
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

if __name__ == '__main__':
    rospy.init_node('app_introduction', anonymous=False)
    #py_trees.logging.level = py_trees.logging.Level.DEBUG

    print("starting..")

    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    print('initialize...')
    behaviour_tree.tick_tock(500)

    rospy.spin()
