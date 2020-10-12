#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from std_msgs.msg import Empty, String, Bool, Header, Float64
from geometry_msgs.msg import Pose, PoseStamped
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseFeedback, PlanExecutePoseResult
from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseFeedback, PlanExecuteNamedPoseResult
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsFeedback, PlanExecutePoseConstraintsResult
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
import shape_msgs.msg
import object_recognition_msgs.srv as object_recognition_srvs
from moveit_msgs.msg import PlanningScene
#from moveit_python import PlanningSceneInterface

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time
from random import *

COLLISION_OBJECT_TOPIC = "/collision_object"
OBJECT_INFORMATION_TOPIC = "/get_object_info"

class MoveitClientNode:
	def __init__(self):
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		self.contraints = Constraints()

		is_initialized = False
		while(not is_initialized):
			try:
				self.group = moveit_commander.MoveGroupCommander("arm")
				is_initialized = True
			except RuntimeError:
				is_initialized = False
				rospy.sleep(0.5)

		rospy.loginfo("Initialized...")
		self.traj_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

		rospy.loginfo("============ Reference planning frame: %s" % self.group.get_planning_frame())
		rospy.loginfo("============ Reference end_effector link: %s" % self.group.get_end_effector_link())

#		self.sub_callback_pointcloud = rospy.Subscriber('/move_group/filtered_cloud', PointCloud2, self.callback_pointcloud)
		self.sub_add_obstacle = rospy.Subscriber('/add_obstacle', String, self.add_obstacle)
		self.sub_del_obstacle = rospy.Subscriber('/del_obstacle', String, self.del_obstacle)
		self.sub_del_all_obstacles = rospy.Subscriber('/del_all_obstacles', String, self.del_all_obstacles)

		self.action_plan_execute_pose = actionlib.SimpleActionServer('/plan_and_execute_pose', PlanExecutePoseAction, execute_cb=self.plan_execute_pose_cb, auto_start = False)
		self.action_plan_execute_pose.start()

		self.action_plan_execute_named_pose = actionlib.SimpleActionServer('/plan_and_execute_named_pose', PlanExecuteNamedPoseAction, execute_cb=self.plan_execute_named_pose_cb, auto_start = False)
		self.action_plan_execute_named_pose.start()

		self.action_plan_execute_pose_w_constraints = actionlib.SimpleActionServer('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction, execute_cb=self.plan_execute_pose_constraints_cb, auto_start = False)
		self.action_plan_execute_pose_w_constraints.start()
		rospy.loginfo('%s ready...'%rospy.get_name())

		self.box_names_arr = []
		self.box_pose = geometry_msgs.msg.PoseStamped()

		self.collision_objects = []


	def callback_pointcloud(self, data):
		assert isinstance(data, PointCloud2)
		for i in range(len(self.box_names_arr)):
			self.scene.remove_world_object(self.box_names_arr[i])
#			print("Remove " + str(self.box_names_arr[i]) + " result : " + str(self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=4, box_name=self.box_names_arr[i])))
		self.box_names_arr = []
		gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
#		time.sleep(1)
		print(type(gen))
		itr = 0
		for p in gen:
			f = random()
#			print(f)
			if f < 0.999:
#				print(f)
				continue
#			print(" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))
			itr += 1
			self.box_pose.header.frame_id = "map"
			self.box_pose.pose.orientation.w = 1.0
			self.box_pose.pose.position.x = p[0]
			self.box_pose.pose.position.y = p[1]
			self.box_pose.pose.position.z = p[2]
			box_name = "point" + str(itr)
			self.scene.add_box(box_name, self.box_pose, size=(0.01, 0.01, 0.01))
			self.box_names_arr.append(box_name)

		start = rospy.get_time()
		seconds = rospy.get_time()
		is_known = False
		print(itr)
#		print("Add box result : " + str(self.wait_for_state_update(box_is_known=True, timeout=4, box_name=box_name)))

	def listen_planningscene(self, msg):
		self.collision_objects = msg.world.collision_objects
		print(self.collision_objects)

	def add_obstacle(self, msg):
# received_data = [position x, position y, position z, box_name, size_x, size_y, size_z]
		received_data = msg.data.split()
		self.box_pose.header.frame_id = "map"

		self.box_pose.pose.orientation.w = 1.0
		self.box_pose.pose.position.x = float(received_data[0])
		self.box_pose.pose.position.y = float(received_data[1])
		self.box_pose.pose.position.z = float(received_data[2])
		box_name = received_data[3]
		self.scene.add_box(box_name, self.box_pose, size=(float(received_data[4]), float(received_data[5]), float(received_data[6])))

		self.box_names_arr.append(box_name)

		start = rospy.get_time()
		seconds = rospy.get_time()
		is_known = False
		print("Add box result : " + str(self.wait_for_state_update(box_is_known=True, timeout=4, box_name=box_name)))

	def del_obstacle(self, msg):
		self.scene.remove_world_object(msg.data)
		print("Remove" + str(msg.data) + "  result : " + str(self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=4, box_name=msg.data)))

	def del_all_obstacles(self, msg):
		print(self.box_names_arr)
		for i in range(len(self.box_names_arr)):
			self.scene.remove_world_object(self.box_names_arr[i])
			print("Remove " + str(self.box_names_arr[i]) + " result : " + str(self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=4, box_name=self.box_names_arr[i])))
		self.box_names_arr = []
		

	def plan_execute_pose_cb(self, goal):
		feedback = PlanExecutePoseFeedback()
		result = PlanExecutePoseResult()
		result.result = True

		self.group.clear_pose_targets()
		self.group.set_start_state_to_current_state()

		#include default arm_base_joint // it is bug that moveit does not exclude passive joint in thier joint trajectory.
		js_base = JointConstraint()
		js_base.joint_name = "arm_base_joint"
		js_base.position = 0.0
		js_base.tolerance_above = 0.001
		js_base.tolerance_below = 0.001
		js_base.weight = 1.0

		self.contraints.name = "constraints"
		self.contraints.joint_constraints.append(js_base)
		self.group.set_path_constraints(self.contraints)

		try:
			self.group.set_pose_target(goal.target_pose)
		except MoveItCommanderException:
			result.result = False
			return

		rospy.loginfo('Planning goal pose...')
		plan1 = self.group.plan()

		if len(plan1.joint_trajectory.points) == 0:
			result.result = False
			return

		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan1)
		self.traj_publisher.publish(display_trajectory)

		rospy.sleep(0.5)

		rospy.loginfo('Start moving...')
		self.group.go(wait=True)
		rospy.sleep(2.0)

		rospy.loginfo('Planning goal pose succeeded.')
		self.action_plan_execute_pose.set_succeeded(result)

	def plan_execute_named_pose_cb(self, goal):
		feedback = PlanExecuteNamedPoseFeedback()
		result = PlanExecuteNamedPoseResult()
		result.result = True

		self.group.clear_pose_targets()
		#self.group.set_start_state_to_current_state()

		#include default arm_base_joint // it is bug that moveit does not exclude passive joint in thier joint trajectory.
		js_base = JointConstraint()
		js_base.joint_name = "arm_base_joint"
		js_base.position = 0.0
		js_base.tolerance_above = 0.001
		js_base.tolerance_below = 0.001
		js_base.weight = 1.0

		self.contraints.name = "constraints"
		self.contraints.joint_constraints.append(js_base)
		self.group.set_path_constraints(self.contraints)

		try:
			self.group.set_named_target(goal.target_name)
		except MoveItCommanderException:
			result.result = False
			self.action_plan_execute_named_pose.set_succeeded(result)
			return

		rospy.loginfo('Planning named [%s] pose...' % goal.target_name)
		plan1 = self.group.plan()

		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan1)
		self.traj_publisher.publish(display_trajectory)

		rospy.sleep(0.5)

		rospy.loginfo('Start moving...')
		self.group.go(wait=True)

		rospy.loginfo('Planning named pose succeeded.')
		self.action_plan_execute_named_pose.set_succeeded(result)

	def plan_execute_pose_constraints_cb(self, goal):
		feedback = PlanExecutePoseConstraintsFeedback()
		result = PlanExecutePoseConstraintsResult()
		result.result = True

		#include default arm_base_joint // it is bug that moveit does not exclude passive joint in thier joint trajectory.
		js_base = JointConstraint()
		js_base.joint_name = "arm_base_joint"
		js_base.position = 0.0
		js_base.tolerance_above = 0.001
		js_base.tolerance_below = 0.001
		js_base.weight = 1.0

		self.contraints.name = "constraints"
		self.contraints.joint_constraints.append(js_base)
		for js in goal.joint_constraints:
			self.contraints.joint_constraints.append(js)
		self.group.set_path_constraints(self.contraints)


		self.group.clear_pose_targets()
		self.group.set_start_state_to_current_state()

		try:
			self.group.set_pose_target(goal.target_pose)
		except MoveItCommanderException:
			result.result = False
			return

		rospy.loginfo('Planning goal pose...')
		plan1 = self.group.plan()

		if len(plan1.joint_trajectory.points) == 0:
			result.result = False
			return

		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan1)
		self.traj_publisher.publish(display_trajectory)

		rospy.sleep(0.5)

		rospy.loginfo('Start moving...')
		self.group.go(wait=True)
		rospy.sleep(2.0)
		self.group.set_path_constraints(None)
		self.contraints.joint_constraints = []

		rospy.loginfo('Planning goal pose succeeded.')
		self.action_plan_execute_pose_w_constraints.set_succeeded(result)

	def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4, box_name=""):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
#		box_name = self.box_name
		scene = self.scene

		## BEGIN_SUB_TUTORIAL wait_for_scene_update
		##
		## Ensuring Collision Updates Are Received
		## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		## If the Python node dies before publishing a collision object update message, the message
		## could get lost and the box will not appear. To ensure that the updates are
		## made, we wait until we see the changes reflected in the
		## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
		## For the purpose of this tutorial, we call this function after adding,
		## removing, attaching or detaching an object in the planning scene. We then wait
		## until the updates have been made or ``timeout`` seconds have passed
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			# Test if the box is in attached objects
			attached_objects = scene.get_attached_objects([box_name])
			is_attached = len(attached_objects.keys()) > 0

			# Test if the box is in the scene.
			# Note that attaching the box will remove it from known_objects
			is_known = box_name in scene.get_known_object_names()

			# Test if we are in the expected state
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True

			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()

		# If we exited the while loop without returning then we timed out
		return False
		## END_SUB_TUTORIAL

	

if __name__ == '__main__':
    rospy.init_node('moveit_client_node', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    m = MoveitClientNode()

    rospy.spin()
