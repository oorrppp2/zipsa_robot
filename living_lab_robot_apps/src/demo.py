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

    head_tilt_up = MoveJoint(name="tilt_up", controller_name="/head/tilt_controller", command=0.0)
    head_tilt_down = MoveJoint(name="tilt_down", controller_name="/head/tilt_controller", command=0.5)

    publish_pause_request = Publish(topic_name="/pause_request", data="pause")
    publish_resume_request = Publish(topic_name="/pause_request", data="resume")

    wait_time1 = WaitForTime(name="delay_1s", time=1.0)
    wait_time05 = WaitForTime(name="delay_0.5s", time=0.5)
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
    # Scene 1 intro  (Introduce and choose the object.)
    #
    intro = py_trees.composites.Sequence("intro")

    wait_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="intro")

    # start_scene1 = Print_message(name="* Move to ready pose *")
    start_mention1 = Print_message(name="* Introduce *")

    scene1_say1 = Say(name="say_hello1", text='안녕하세요? 저는 한국과학기술연구원의 로봇, 집사입니다.')

    done_scene_1 = Publish(topic_name="/wait_done_scene", data="scene_1_done")
    intro.add_children(
        [wait_intro,
         start_mention1,
         scene1_say1,
         done_scene_1,
         ]
    )


    #
    # Scene 2 Move to user
    #
    move_to_user = py_trees.composites.Sequence("move_to_user")

    wait_move_to_user = py_trees_ros.subscribers.CheckData(name="wait_move_to_user", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="move_to_user")

    move_to_user_mention1 = Print_message(name="* move_to_user *")
    goal_user = move_base_msgs.msg.MoveBaseGoal()
    goal_user.target_pose.header.frame_id = "map"
    goal_user.target_pose.header.stamp = rospy.Time.now()

    goal_user.target_pose.pose.position.x = 1.575
    goal_user.target_pose.pose.position.y = -2.36

    goal_user.target_pose.pose.orientation.x = 0
    goal_user.target_pose.pose.orientation.y = 0
    goal_user.target_pose.pose.orientation.z = -0.701
    goal_user.target_pose.pose.orientation.w = 0.713

    move_to_user_action = py_trees_ros.actions.ActionClient(
        name="move to user",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_user
    )

    done_scene_2 = Publish(topic_name="/wait_done_scene", data="scene_2_done")
    move_to_user.add_children(
        [wait_move_to_user,
         move_to_user_mention1,
         arm_put_in,
         head_tilt_up,
         wait_time1,
         wait_time1,
         move_to_user_action,
         done_scene_2,
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
         order_object,
         ]
    )

    #
    # Scene 4 Move to forward of shelf
    #
    move_to_shelf = py_trees.composites.Sequence("move_to_shelf")

    wait_move_to_shelf = py_trees_ros.subscribers.CheckData(name="wait_move_to_shelf", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="move_to_shelf")

    move_to_shelf_mention1 = Print_message(name="* Move_to_shelf *")
    goal_shelf = move_base_msgs.msg.MoveBaseGoal()
    goal_shelf.target_pose.header.frame_id = "map"
    goal_shelf.target_pose.header.stamp = rospy.Time.now()

    goal_shelf.target_pose.pose.position.x = 0.433
    goal_shelf.target_pose.pose.position.y = -0.866

    goal_shelf.target_pose.pose.orientation.x = 0
    goal_shelf.target_pose.pose.orientation.y = 0
    goal_shelf.target_pose.pose.orientation.z = 1.0
    goal_shelf.target_pose.pose.orientation.w = 0.

    move_to_shelf_action = py_trees_ros.actions.ActionClient(
        name="move to shelf",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_shelf
    )
    arm_pull_out = Fold_arm("Pull out", 0)
    done_scene_4 = Publish(topic_name="/wait_done_scene", data="scene_4_done")

    move_to_shelf.add_children(
        [wait_move_to_shelf,
         move_to_shelf_mention1,
         move_to_shelf_action,
         arm_pull_out,
         head_tilt_down,
         done_scene_4,
         ]
    )

    #
    # Scene 5 Find_target  (Find object.)
    #

    find_target = py_trees.composites.Sequence("find_target")

    wait_find_target= py_trees_ros.subscribers.CheckData(name="wait_find_target", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="find_target")

#    start_scene3 = Print_message(name="* Scene  *")
    find_target_mention1 = Print_message(name="Finding target ...")

    find_object = ObjectDetectionActionClient(
        name="find_object",
        action_namespace="/object_detect",
        action_spec=ObjectDetectAction,
        action_goal=ObjectDetectGoal()
    )
    done_scene_5 = Publish(topic_name="/wait_done_scene", data="scene_5_done")

    find_target.add_children(
        [wait_find_target,
         find_target_mention1,
         publish_resume_request,
         find_object,
         publish_resume_request,
         find_object,
         done_scene_5,
         ]
    )

    #
    # Scene 6 Arm_control  (Move arm to the target object to grasp it.)
    # x축으로 3cm 앞, z축으로 5cm 위를 경유하여
    # x축으로 3cm 더 깊이(첫 경유보다 6cm 깊이), z축 center 기준으로 -1cm 지점으로 move.
    #

    arm_control = py_trees.composites.Sequence("arm_control")
    wait_arm_control = py_trees_ros.subscribers.CheckData(name="wait_arm_control", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="arm_control")
    arm_control_mention1 = Print_message(name="* Arm_control *")

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
    done_scene_6 = Publish(topic_name="/wait_done_scene", data="scene_6_done")

    arm_control.add_children(
        [wait_arm_control,
         arm_control_mention1,
         publish_pause_request,
         # find_object,
         move_manipulator_to_grasp_add_offset,
         wait_time1,
         wait_time1,
         move_manipulator_to_grasp,
         #move_manipulator_to_grasp_ready,
         done_scene_6,
         ]
    )

    #
    # Scene 7 Grasp the object.  (Gripper close and arm position into 'grasp_done')
    # 물건을 잡고 10cm 들어 올림.
    #
    grasp_object = py_trees.composites.Sequence("grasp_object")

    wait_grasp_object = py_trees_ros.subscribers.CheckData(name="wait_grasp_object", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="grasp_object")

    grasp_object_mention1 = Print_message(name="* Closing the gripper *")

    elevation_up_action = Elevation_up(target_pose=0.1)
    elevation_down_20cm_action = Elevation_up(target_pose=-0.2)
    done_scene_7 = Publish(topic_name="/wait_done_scene", data="scene_7_done")

    grasp_object.add_children(
        [wait_grasp_object,
         grasp_object_mention1,
         gripper_close,
         wait_time1,
         elevation_up_action,
         wait_time1,
         move_manipulator_to_grasp_done,
         publish_resume_request,
         wait_time1,
         move_manipulator_to_grasp_ready,
         done_scene_7,
         ]
    )

    #
    # Scene 8 Move to tea table.
    #
    move_to_tea_table = py_trees.composites.Sequence("move_to_tea_table")

    wait_move_to_tea_table = py_trees_ros.subscribers.CheckData(name="wait_move_to_tea_table", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="move_to_tea_table")

    move_to_tea_table_mention1 = Print_message(name="* Move_to_tea_table *")
	# coffee table
    goal_tea_table = move_base_msgs.msg.MoveBaseGoal()
    goal_tea_table.target_pose.header.frame_id = "map"
    goal_tea_table.target_pose.header.stamp = rospy.Time.now()

    goal_tea_table.target_pose.pose.position.x = 2.422
    goal_tea_table.target_pose.pose.position.y = -2.4

    goal_tea_table.target_pose.pose.orientation.x = 0
    goal_tea_table.target_pose.pose.orientation.y = 0
    goal_tea_table.target_pose.pose.orientation.z = -0.701
    goal_tea_table.target_pose.pose.orientation.w = 0.713

    move_to_tea_table_action = py_trees_ros.actions.ActionClient(
        name="move to tea table",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_tea_table
    )
    request_nonremoval_point_cloud = Publish(topic_name="/remove_points_request", data="full_pc")
    done_scene_8 = Publish(topic_name="/wait_done_scene", data="scene_8_done")

    move_to_tea_table.add_children(
        [wait_move_to_tea_table,
         move_to_tea_table_mention1,
         move_to_tea_table_action,
         request_nonremoval_point_cloud,
         done_scene_8,
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
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[0.0, 0.0, 0.25]},
        mode="put"
    )

    # elevation_down_20cm_action = Elevation_up(target_pose=-0.2)
    elevation_down_action = Elevation_up(target_pose=-0.086)
    elevation_up_action = Elevation_up(target_pose=0.3)
    # done_scene_9 = Publish(topic_name="/wait_done_scene", data="scene_9_done")

    put_object.add_children(
        [wait_put_object,
         put_object_mention1,
        #  elevation_down_20cm_action,
        #  wait_time1,
        #  wait_time1,
         move_manipulator_to_put_down,
         wait_time1,
         wait_time1,
         elevation_down_action,
         wait_time1,
         wait_time1,
         gripper_open,
         elevation_up_action,
#         move_manipulator_to_grasp_done,
         move_manipulator_to_home,
         done_scene_1,      # move to user
         ]
    )

    #
    # Scene 10 Go home. (Move to home position)
    #

    go_home = py_trees.composites.Sequence("go_home")

    wait_go_home = py_trees_ros.subscribers.CheckData(name="wait_go_home", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="go_home")

    go_home_mention = Print_message(name="* go_home *")

    goal_home = move_base_msgs.msg.MoveBaseGoal()
    goal_home.target_pose.header.frame_id = "map"
    goal_home.target_pose.header.stamp = rospy.Time.now()
    goal_home.target_pose.pose.position.x = 0
    goal_home.target_pose.pose.position.y = 0

    goal_home.target_pose.pose.orientation.x = 0
    goal_home.target_pose.pose.orientation.y = 0
    goal_home.target_pose.pose.orientation.z = 0
    goal_home.target_pose.pose.orientation.w = 1.0

    move_to_home = py_trees_ros.actions.ActionClient(
        name="move to home",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_home
    )
    done_scene_10 = Publish(topic_name="/wait_done_scene", data="scene_10_done")

    go_home.add_children(
        [wait_go_home,
         go_home_mention,
         arm_put_in,
         head_tilt_up,
         move_to_home,
         done_scene_10,
         ]
    )


    #
    # Scene 11 Speech1 (Speech Case1)
    #
    speech1 = py_trees.composites.Sequence("speech1")

    wait_speech1 = py_trees_ros.subscribers.CheckData(name="wait_speech1", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="speech1")

    start_speech1 = Print_message(name="* Speech1 *")

    scene11_speech1 = Say(name="say_speech1", text='첫번째 케이스에 대한 문장을 출력합니다.')

    done_speech1 = Publish(topic_name="/wait_done_scene", data="scene_speech1_done")

    speech1.add_children(
        [wait_speech1,
         start_speech1,
         scene11_speech1,
         done_speech1,
         ]
    )


    #
    # Scene 12 Speech2 (Speech Case2)
    #
    speech2 = py_trees.composites.Sequence("speech2")

    wait_speech2 = py_trees_ros.subscribers.CheckData(name="wait_speech2", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="speech2")

    start_speech2 = Print_message(name="* Speech2 *")

    scene12_speech2 = Say(name="say_speech2", text='두번째 케이스에 대한 문장을 출력합니다.')

    done_speech2 = Publish(topic_name="/wait_done_scene", data="scene_speech2_done")

    speech2.add_children(
        [wait_speech2,
         start_speech2,
         scene12_speech2,
         done_speech2,
         ]
    )


    #
    # Scene 13 Speech3 (Speech Case3)
    #
    speech3 = py_trees.composites.Sequence("speech3")

    wait_speech3 = py_trees_ros.subscribers.CheckData(name="wait_speech3", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="speech3")

    start_speech3 = Print_message(name="* Speech3 *")

    scene13_speech3 = Say(name="say_speech3", text='세번째 케이스에 대한 문장을 출력합니다.')

    done_speech3 = Publish(topic_name="/wait_done_scene", data="scene_speech3_done")

    speech3.add_children(
        [wait_speech3,
         start_speech3,
         scene13_speech3,
         done_speech3,
         ]
    )


    #
    # Scene 14 Question1 (Question Case1)
    #
    question1 = py_trees.composites.Sequence("question1")

    wait_question1 = py_trees_ros.subscribers.CheckData(name="wait_question1", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="question1")

    start_question1 = Print_message(name="* Question1 *")

    scene14_question1 = Say(name="say_question1", text='사용자에게 물어볼 첫번째 질문을 출력합니다.')

    done_question1 = Publish(topic_name="/wait_done_scene", data="scene_question1_done")

    question1.add_children(
        [wait_question1,
         start_question1,
         scene14_question1,
         done_question1,
         ]
    )


    #
    # Scene 15 Question2 (Question Case2)
    #
    question2 = py_trees.composites.Sequence("question2")

    wait_question2 = py_trees_ros.subscribers.CheckData(name="wait_question2", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="question2")

    start_question2 = Print_message(name="* Question2 *")

    scene15_question2 = Say(name="say_question2", text='사용자에게 물어볼 두번째 질문을 출력합니다.')

    done_question2 = Publish(topic_name="/wait_done_scene", data="scene_question2_done")

    question2.add_children(
        [wait_question2,
         start_question2,
         scene15_question2,
         done_question2,
         ]
    )


    #
    # Scene 16 Question3 (Question Case3)
    #
    question3 = py_trees.composites.Sequence("question3")

    wait_question3 = py_trees_ros.subscribers.CheckData(name="wait_question3", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="question3")

    start_question3 = Print_message(name="* Question3 *")

    scene16_question3 = Say(name="say_question3", text='사용자에게 물어볼 세번째 질문을 출력합니다.')

    done_question3 = Publish(topic_name="/wait_done_scene", data="scene_question3_done")

    question3.add_children(
        [wait_question3,
         start_question3,
         scene16_question3,
         done_question3,
         ]
    )


    #
    # Scene 17 Speech Test (Speech Test)
    #
    speech_test = py_trees.composites.Sequence("speech_test")

    wait_speech_test = py_trees_ros.subscribers.CheckData(name="wait_speech_test", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="speech_test")

    start_speech_test = Print_message(name="* Speech Test *")

    scene17_speech_test = Say(name="say_speech_test", text='집에 가서 유튜브 보며 치맥하고 싶어요. 퇴근시켜 주세요.')

    done_speech_test = Publish(topic_name="/wait_done_scene", data="scene_speech_test_done")

    speech_test.add_children(
        [wait_speech_test,
         start_speech_test,
         scene17_speech_test,
         done_speech_test,
         ]
    )


    #
    # Scene 18 Move to Living Room
    #

    move_to_living_room = py_trees.composites.Sequence("move_to_living_room")

    wait_move_to_living_room = py_trees_ros.subscribers.CheckData(name="wait_move_to_living_room", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="move_to_living_room")

    start_move_to_living_room = Print_message(name="* Move to Living Room *")

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

    move_to_living_room_order = py_trees_ros.actions.ActionClient(
        name="move to living room",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_living_room
    )

    done_move_to_living_room = Publish(topic_name="/wait_done_scene", data="scene_move_to_living_room_done")

    move_to_living_room.add_children(
        [ wait_move_to_living_room,
          start_move_to_living_room,
          move_to_living_room_order,
          done_move_to_living_room]
    )


    #
    # Scene 19 Move to Kitchen
    #

    move_to_kitchen = py_trees.composites.Sequence("move_to_kitchen")

    wait_move_to_kitchen = py_trees_ros.subscribers.CheckData(name="wait_move_to_kitchen", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="move_to_kitchen")

    start_move_to_kitchen = Print_message(name="* Move to Kitchen *")

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

    move_to_kitchen_order = py_trees_ros.actions.ActionClient(
        name="move to kitchen",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_kitchen
    )

    done_move_to_kitchen = Publish(topic_name="/wait_done_scene", data="scene_move_to_kitchen")

    move_to_kitchen.add_children(
        [ wait_move_to_kitchen,
          start_move_to_kitchen,
          move_to_kitchen_order,
          done_move_to_kitchen ]
    )


    ###
    ### finish demo
    ###
    finish_demo = py_trees.composites.Sequence("finish_demo")
    wait_finish_demo = py_trees_ros.subscribers.CheckData(name="wait_finish_demo", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="finish_demo")
    finish_demo_mention1 = Print_message(name="* Finish_demo *")

    finish_demo.add_children(
        [wait_finish_demo,
         finish_demo_mention1,
         head_tilt_down,
         move_manipulator_to_home,
         arm_put_in,
         done_scene,
         ]
    )

    elevation_up = py_trees.composites.Sequence("Elevation_up")
    wait_elevation_up = py_trees_ros.subscribers.CheckData(name="wait_elevation_up", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="elevation_up")
    elevation_up_mention1 = Print_message(name="* Elevation_up *")

#			Elevation_up : desired elevation position = current position + target_pose
    elevation_up_10cm_action = Elevation_up(target_pose=0.1)
    elevation_down_10cm_action = Elevation_up(target_pose=-0.1)

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
         done_scene,
         ]
    )



    ###
    ### add childern
    ###
    root.add_children([gripper_open_cmd, intro, order_the_target, move_to_user, move_to_shelf, find_target, arm_control, move_to_tea_table, grasp_object, go_home, finish_demo, put_object, elevation_up, elevation_down, speech1, speech2, speech3, question1, question2, question3, speech_test, move_to_living_room, move_to_kitchen])
    # root.add_children([scene1, scene3, scene4, scene5, scene6, scene7])
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
