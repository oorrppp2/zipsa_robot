#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseFeedback, PlanExecutePoseResult
from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseFeedback, PlanExecuteNamedPoseResult
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsFeedback, PlanExecutePoseConstraintsResult
from moveit_msgs.msg import RobotState, Constraints, JointConstraint

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

        self.action_plan_execute_pose = actionlib.SimpleActionServer('/plan_and_execute_pose', PlanExecutePoseAction, execute_cb=self.plan_execute_pose_cb, auto_start = False)
        self.action_plan_execute_pose.start()

        self.action_plan_execute_named_pose = actionlib.SimpleActionServer('/plan_and_execute_named_pose', PlanExecuteNamedPoseAction, execute_cb=self.plan_execute_named_pose_cb, auto_start = False)
        self.action_plan_execute_named_pose.start()

        self.action_plan_execute_pose_w_constraints = actionlib.SimpleActionServer('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction, execute_cb=self.plan_execute_pose_constraints_cb, auto_start = False)
        self.action_plan_execute_pose_w_constraints.start()
        rospy.loginfo('%s ready...'%rospy.get_name())

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
        #


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
        #


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


        self.group.set_path_constraints(None)
        self.contraints.joint_constraints = []


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


if __name__ == '__main__':
    rospy.init_node('moveit_client_node', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    m = MoveitClientNode()

    rospy.spin()