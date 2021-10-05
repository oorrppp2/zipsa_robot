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

from behaviors.lamp_control import *
from behaviors.utils import *
from behaviors.wait_time import *

global Point_data
global Point_flag

def create_root():

    root = py_trees.composites.Parallel("demo")
    # done_scene = DonePlayScene(name="done_scene")

    lamp_mode0 = LampControl(name="lamp_mode_0", mode=0, args="{}")
    lamp_mode1 = LampControl(name="lamp_mode_1", mode=1, args="{}")
    lamp_mode2 = LampControl(name="lamp_mode_2", mode=2, args="{}")
    lamp_mode3r = LampControl(name="lamp_mode_3r", mode=3, args="{\"color\": \"r\"}")
    lamp_mode3g = LampControl(name="lamp_mode_3g", mode=3, args="{\"color\": \"g\"}")
    lamp_mode3b = LampControl(name="lamp_mode_3b", mode=3, args="{\"color\": \"b\"}")

    wait_time1 = WaitForTime(name="delay_1s", time=1.0)
    wait_time05 = WaitForTime(name="delay_0.5s", time=0.5)
    #
    # lamp_test  (Testing lamp)
    #
    lamp_test_scene = py_trees.composites.Sequence("lamp_test_scene")

    wait_lamp_test_scene = py_trees_ros.subscribers.CheckData(name="wait_lamp_test_scene", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="lamp_test_scene")

    start_lamp_test = Print_message(name="* Start testing lamp *")
    done_lamp_test = Print_message(name="* Testing lamp done *")

    lamp_test_scene.add_children(
        [wait_lamp_test_scene,
        start_lamp_test,
        lamp_mode0,
        wait_time1,
        wait_time1,
         lamp_mode1,
        wait_time1,
        wait_time1,
         lamp_mode2,
        wait_time1,
        wait_time1,
         done_lamp_test,
         ]
    )

    # root.add_children([grasp_bowl_ready, pour_juice, shake_arm, lamp_test_scene,gripper_open_cmd, order_the_target, put_object, grab_target])
    # root.add_children([scene1, scene3, scene4, scene5, scene6, scene7])
    root.add_children([lamp_test_scene])
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
