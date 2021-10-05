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

    lamp_mode0 = LampControl(name="lamp_mode_0", mode=0, args="{}")
    lamp_mode1 = LampControl(name="lamp_mode_1", mode=1, args="{}")
    lamp_mode2 = LampControl(name="lamp_mode_2", mode=2, args="{}")
    lamp_mode3r = LampControl(name="lamp_mode_3r", mode=3, args="{\"color\": \"r\"}")
    lamp_mode3g = LampControl(name="lamp_mode_3g", mode=3, args="{\"color\": \"g\"}")
    lamp_mode3b = LampControl(name="lamp_mode_3b", mode=3, args="{\"color\": \"b\"}")

    gripper_close = MoveJoint(name="gripper_close", controller_name="/body/gripper_controller", command=0.0)
    gripper_open = MoveJoint(name="gripper_open", controller_name="/body/gripper_controller", command=1.0)

    goal_grap_ready = PlanExecuteNamedPoseGoal()
    goal_grap_ready.target_name ="grasp_ready"
    move_manipulator_to_grasp_ready = py_trees_ros.actions.ActionClient(
        name="move_manipulator_to_grasp_ready",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=goal_grap_ready
    )

    goal_grap_done = PlanExecuteNamedPoseGoal()
    goal_grap_done.target_name ="grasp_done"
    move_manipulator_to_grasp_done = py_trees_ros.actions.ActionClient(
        name="move_manipulator_to_grasp_done",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=goal_grap_done
    )

    goal_home = PlanExecuteNamedPoseGoal()
    goal_home.target_name ="home"
    move_manipulator_to_home = py_trees_ros.actions.ActionClient(
        name="move_manipulator_to_home",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=goal_home
    )

    shake_arm_left = PlanExecuteNamedPoseGoal()
    shake_arm_left.target_name ="shake_arm_left"
    move_manipulator_to_left = py_trees_ros.actions.ActionClient(
        name="shake_arm_left",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=shake_arm_left
    )

    shake_arm_right = PlanExecuteNamedPoseGoal()
    shake_arm_right.target_name ="shake_arm_right"
    move_manipulator_to_right = py_trees_ros.actions.ActionClient(
        name="shake_arm_right",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=shake_arm_right
    )

    pour_juice_ready = PlanExecuteNamedPoseGoal()
    pour_juice_ready.target_name ="pour_juice_ready"
    move_manipulator_to_pour_juice_ready = py_trees_ros.actions.ActionClient(
        name="pour_juice_ready",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=pour_juice_ready
    )

    pour_juice_into_cup = PlanExecuteNamedPoseGoal()
    pour_juice_into_cup.target_name ="pour_juice_into_cup"
    move_manipulator_to_pour_juice_into_cup = py_trees_ros.actions.ActionClient(
        name="pour_juice_into_cup",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=pour_juice_into_cup
    )

    grasp_bowl_ready = PlanExecuteNamedPoseGoal()
    grasp_bowl_ready.target_name ="grasp_bowl_ready"
    move_manipulator_to_grasp_bowl_ready = py_trees_ros.actions.ActionClient(
        name="grasp_bowl_ready",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=grasp_bowl_ready
    )

    head_tilt_up = MoveJoint(name="tilt_up", controller_name="/head/tilt_controller", command=0.0)
    head_tilt_down = MoveJoint(name="tilt_down", controller_name="/head/tilt_controller", command=0.5)

    publish_pause_request = Publish(topic_name="/pause_request", data="pause")
    publish_resume_request = Publish(topic_name="/pause_request", data="resume")
    publish_putting_down_request = Publish(topic_name="/pause_request", data="putting_down")

    wait_time1 = WaitForTime(name="delay_1s", time=1.0)
    wait_time05 = WaitForTime(name="delay_0.5s", time=0.5)

    arm_pull_out = Fold_arm("Pull out", 0)

    # Shaking arm
    shake_arm = py_trees.composites.Sequence("shake_arm")

    wait_shake_arm = py_trees_ros.subscribers.CheckData(name="wait_shake_arm", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="shake_arm")

    shake_arm.add_children(
        [wait_shake_arm,
         move_manipulator_to_left,
         move_manipulator_to_right,
         move_manipulator_to_left,
         move_manipulator_to_right,
         move_manipulator_to_left,
         move_manipulator_to_right,
         ]
    )

    # Pour juice into a cup
    pour_juice = py_trees.composites.Sequence("pour_juice")

    wait_pour_juice = py_trees_ros.subscribers.CheckData(name="wait_pour_juice", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="pour_juice")

    pour_juice.add_children(
        [wait_pour_juice,
        #  move_manipulator_to_pour_juice_ready,
         move_manipulator_to_pour_juice_into_cup,
         wait_time1,
         wait_time05,
         move_manipulator_to_pour_juice_ready,
         ]
    )

    # Grasp bowl
    grasp_bowl_ready = py_trees.composites.Sequence("grasp_bowl_ready")

    wait_grasp_bowl_ready = py_trees_ros.subscribers.CheckData(name="wait_grasp_bowl_ready", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="grasp_bowl_ready")

    move_manipulator_to_grasp_bowl = GraspBowlActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=0.0,
        y_offset=0.0,
        z_offset=-0.13,
        constraint=True,
        joint={'body_rotate_joint':[0.0, 5 * math.pi / 180.0, 5 * math.pi / 180.0],}
    )

    grasp_bowl_ready.add_children(
        [wait_grasp_bowl_ready,
        #  move_manipulator_to_grasp_bowl_ready,
         move_manipulator_to_grasp_bowl,
         gripper_close,
         wait_time05,
         move_manipulator_to_grasp_bowl_ready,
         ]
    )

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

    #
    # gripper_open  (gripper_open arm)
    #
    gripper_open_cmd = py_trees.composites.Sequence("gripper_open")

    wait_gripper_open = py_trees_ros.subscribers.CheckData(name="wait_gripper_open", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="gripper_open")

    start_gripper_open = Print_message(name="* Opening the gripper *")

    gripper_open_cmd.add_children(
        [wait_gripper_open,
         start_gripper_open,
         gripper_open,
         done_scene,
         ]
    )

    #
    # fold_arm  (fold arm)
    #
    fold_arm = py_trees.composites.Sequence("fold_arm")

    wait_fold_arm = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="fold_arm")

    start_fold_arm = Print_message(name="* Folding the arm *")
    arm_put_in = Fold_arm("Put in", -0.15)

    fold_arm.add_children(
        [wait_fold_arm,
         start_fold_arm,
         move_manipulator_to_home,
         arm_put_in,
         done_scene,
         ]
    )

    #
    # Scene 3 Order the target
    #
    order_the_target = py_trees.composites.Sequence("order_the_target")

    wait_order_the_target = py_trees_ros.subscribers.CheckData(name="wait_order_the_target", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="order_the_target")

    order_mention1 = Print_message(name="Choose cup / bottle / milk  or go home?")
    order_object = OrderActionClient(
        name="order_received",
        # action_namespace="/sst_order_received",
        action_namespace="/order_received",
        action_spec=ReceiveTargetAction,
        action_goal=ReceiveTargetGoal()
    )

    order_the_target.add_children(
        [wait_order_the_target,
         order_mention1,
         order_object,
        #  order_target_action
         ]
    )

    #
    # Scene 5 Find_target  (Find object.)
    #

    grab_target = py_trees.composites.Sequence("find_target")

    wait_grab_target= py_trees_ros.subscribers.CheckData(name="wait_grab_target", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="grab_target")

#    start_scene3 = Print_message(name="* Scene  *")
    find_target_mention1 = Print_message(name="Finding target ...")

    find_object = ObjectDetectionActionClient(
        name="find_object",
        action_namespace="/object_detect",
        action_spec=ObjectDetectAction,
        action_goal=ObjectDetectGoal()
    )

    arm_control_mention1 = Print_message(name="* Arm_control *")
    rotate_body_joint_to_heading_target = Body_Rotate()
    move_manipulator_to_grasp_add_offset = GraspActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=-0.02,
        y_offset=0,
        z_offset=0.05,
        constraint=True,
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[0.0, 0.0, 0.25]}
#        joint=["arm1_joint", "arm6_joint"]
    )
    move_manipulator_to_grasp = GraspActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=0.03,
        y_offset=0,
#        z_offset=-0.01,
        constraint=True,
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[0.0, 0.05, 0.25]}
    )

    grasp_object_mention1 = Print_message(name="* Closing the gripper *")
    elevation_up_action = Elevation_up(target_pose=0.1)
    elevation_down_20cm_action = Elevation_up(target_pose=-0.2)

    grab_target.add_children(
        [wait_grab_target,
         find_target_mention1,
         arm_pull_out,
         head_tilt_down,
         move_manipulator_to_grasp_done,
         publish_resume_request,
         find_object,
         arm_control_mention1,
         publish_pause_request,
         wait_time1,
         wait_time05,
         rotate_body_joint_to_heading_target,
         move_manipulator_to_grasp_add_offset,
        #  wait_time1,
         wait_time1,
         move_manipulator_to_grasp,
         grasp_object_mention1,
         wait_time1,
         gripper_close,
         elevation_up_action,
        #  wait_time1,
         move_manipulator_to_grasp_done,
        #  wait_time1,
         publish_resume_request,
         done_scene,
         ]
    )


    #
    # Scene 9 Put the object down.
    # 목표 지점 위 8cm 위치를 경유한 뒤 7cm elevation을 내림.
    #
    put_object = py_trees.composites.Sequence("put_object")

    wait_put_object = py_trees_ros.subscribers.CheckData(name="wait_put_object", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="put_object")

    put_object_mention1 = Print_message(name="* Putting down the object*")

    move_manipulator_to_put_down = GraspActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        constraint=True,
        joint={
            'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm6_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'body_rotate_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[0.0, 0.0, 0.25]
            },
        mode="put"
    )

    octomap_clear = Clear_octomap()

    # elevation_down_20cm_action = Elevation_up(target_pose=-0.2)
    elevation_down_action = Elevation_up(target_pose=-0.086)
    elevation_up_action = Elevation_up(target_pose=0.3)
    # done_scene_9 = Publish(topic_name="/wait_done_scene", data="scene_9_done")

    put_object.add_children(
        [wait_put_object,
         put_object_mention1,
         octomap_clear,
         wait_time1,
         move_manipulator_to_put_down,
        #  wait_time1,
         elevation_down_action,
         wait_time1,
         wait_time05,
         gripper_open,
         elevation_up_action,
         wait_time1,
         wait_time1,
        #  move_manipulator_to_grasp_ready,
         move_manipulator_to_home,
         wait_time05,
         done_scene,
         ]
    )

    #
    # Elevation up & down
    #

    elevation_up = py_trees.composites.Sequence("Elevation_up")
    wait_elevation_up = py_trees_ros.subscribers.CheckData(name="wait_elevation_up", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="elevation_up")
    elevation_up_mention1 = Print_message(name="* Elevation_up *")

#			Elevation_up : desired elevation position = current position + target_pose
    elevation_up_10cm_action = Elevation_up(target_pose=0.1)
    elevation_up_20cm_action = Elevation_up(target_pose=0.2)
    elevation_down_10cm_action = Elevation_up(target_pose=-0.1)
    elevation_down_20cm_action = Elevation_up(target_pose=-0.2)

    elevation_up.add_children(
        [wait_elevation_up,
         elevation_up_mention1,
         elevation_up_10cm_action,
         done_scene,
         ]
    )

    elevation_down = py_trees.composites.Sequence("Elevation_down")
    wait_elevation_down = py_trees_ros.subscribers.CheckData(name="wait_elevation_down", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="elevation_down")
    elevation_down_mention1 = Print_message(name="* Elevation_down *")

#			Elevation_up : desired elevation position = current position + target_pose

    elevation_down.add_children(
        [wait_elevation_down,
         elevation_down_mention1,
         elevation_down_10cm_action,
         wait_time1,
         ]
    )

    put_bowl = py_trees.composites.Sequence("put_bowl")
    wait_put_bowl = py_trees_ros.subscribers.CheckData(name="wait_put_bowl", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="put_bowl")
    put_bowl_mention1 = Print_message(name="* Put bowl *")

#			Elevation_up : desired elevation position = current position + target_pose

    put_bowl.add_children(
        [wait_put_bowl,
         put_bowl_mention1,
         elevation_down_20cm_action,
         wait_time1,
         wait_time1,
         wait_time1,
         gripper_open,
         wait_time1,
         elevation_up_20cm_action,
         ]
    )

    ###
    ### add childern
    ###
    root.add_children([grasp_bowl_ready, pour_juice, shake_arm, lamp_test_scene,gripper_open_cmd, order_the_target, put_object, grab_target, elevation_down, elevation_up, put_bowl])
    # root.add_children([scene1, scene3, scene4, scene5, scene6, scene7])
    # root.add_children([lamp_test_scene])
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
