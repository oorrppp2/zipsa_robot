#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal
from living_lab_robot_perception.msg import PoseEstimationAction, PoseEstimationResult, PoseEstimationGoal
import tf
from tf.transformations import *
from std_srvs.srv import Empty
from control_msgs.srv import QueryTrajectoryState

# from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
import geometry_msgs.msg
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import String, Float64
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2

# import pcl
# import pcl_helper


def main(argv):
    gripper_control_pub = rospy.Publisher('/body/gripper_controller/command', Float64, queue_size=10)
    # root_dir = rospy.get_param("root_dir", default="")
    plan_client = actionlib.SimpleActionClient('/plan_and_execute_pose', PlanExecutePoseAction)
    plan_client.wait_for_server()

    constrained_plan_client = actionlib.SimpleActionClient('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction)
    constrained_plan_client.wait_for_server()

    pose_client = actionlib.SimpleActionClient('/pose_estimation', PoseEstimationAction)
    pose_client.wait_for_server()

    elevation_up_client = actionlib.SimpleActionClient('/body/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    elevation_up_client.wait_for_server()

    named_client = actionlib.SimpleActionClient('/plan_and_execute_named_pose', PlanExecuteNamedPoseAction)
    named_client.wait_for_server()

    rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    # clear_octomap()
    # exit(0)
    remove_switching_pub = rospy.Publisher('/remove_points_switch', String, queue_size=1)

    goal = PlanExecutePoseGoal()
    sub_goal = PlanExecutePoseGoal()
    tf_buffer = tf2_ros.Buffer()
    br = tf2_ros.TransformBroadcaster()

    listener = tf.TransformListener()

    r = rospy.Rate(10)
    target_id = 11
    root_dir = '/home/robot/catkin_ws/src/zipsa_robot/living_lab_robot_perception'
    initial_poses = np.load("{0}/predefined_poses/{1}.npy".format(root_dir, target_id))

    pose_goal = PoseEstimationGoal()
    pose_goal.target_id = target_id
    pose_client.send_goal(pose_goal)
    pose_client.wait_for_result()

    estimated_pose = pose_client.get_result().estimated_pose

    estimated_pose = np.reshape(estimated_pose, (4,4))
    print(estimated_pose)

    for initial_pose in initial_poses:
        remove_switching_pub.publish("off")
        print("initial_pose")
        print(initial_pose)
        # if initial_pose[2,3] > 0.:
        #     continue
        # if initial_pose[0,3] < 0:
        #     continue
        if np.sum(initial_pose) == 0:
            continue
        M = np.matmul(estimated_pose, initial_pose)
        quat = quaternion_from_matrix(M)

        try:
            detect_pose = geometry_msgs.msg.TransformStamped()
            detect_pose.header.frame_id = "camera_color_optical_frame"
            detect_pose.header.stamp = rospy.Time.now()
            detect_pose.child_frame_id = "object_coordinate"
            detect_pose.transform.translation.x = M[0,3]
            detect_pose.transform.translation.y = M[1,3]
            detect_pose.transform.translation.z = M[2,3]

            detect_pose.transform.rotation.x = quat[0]
            detect_pose.transform.rotation.y = quat[1]
            detect_pose.transform.rotation.z = quat[2]
            detect_pose.transform.rotation.w = quat[3]

            count = 0
            while 1:
                count += 1
                detect_pose.header.stamp = rospy.Time.now()
                br.sendTransform(detect_pose)
                r.sleep()
                try:
                    (trans,rot) = listener.lookupTransform('/base_footprint', '/object_coordinate', rospy.Time(0))
                    break
                except:
                    continue

            # print("trans: ", trans)
            # print("rot: ", rot)
            # convert detect_pose with reference base_footprint

            goal.target_pose.header.frame_id = "base_footprint"
            goal.target_pose.pose.position.x = trans[0]
            goal.target_pose.pose.position.y = trans[1]
            goal.target_pose.pose.position.z = trans[2]

            goal.target_pose.pose.orientation.x = rot[0]
            goal.target_pose.pose.orientation.y = rot[1]
            goal.target_pose.pose.orientation.z = rot[2]
            goal.target_pose.pose.orientation.w = rot[3]

            roll, pitch, yaw = euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])
            # print(roll, pitch, yaw)

            if yaw < -math.pi / 2.0:
                yaw += math.pi
            elif yaw > math.pi / 2.0:
                yaw -= math.pi
            quat = quaternion_from_euler(roll, pitch, yaw)

            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]

            R = quaternion_matrix(quat)[:3,:3]
            XL2 = np.matmul(R.T, trans)
            XL2[0] -= 0.1

            sub_trans = np.matmul(R, XL2)

            sub_goal.target_pose.header.frame_id = "base_footprint"
            sub_goal.target_pose.pose.position.x = sub_trans[0]
            sub_goal.target_pose.pose.position.y = sub_trans[1]
            sub_goal.target_pose.pose.position.z = sub_trans[2]

            sub_goal.target_pose.pose.orientation.x = quat[0]
            sub_goal.target_pose.pose.orientation.y = quat[1]
            sub_goal.target_pose.pose.orientation.z = quat[2]
            sub_goal.target_pose.pose.orientation.w = quat[3]

        except ValueError:
            quit()

        print(sub_goal)
        constrained_plan_sub_goal = PlanExecutePoseConstraintsGoal()
        constrained_plan_sub_goal.target_pose = sub_goal.target_pose

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'arm4_joint'
        joint_constraint.position = 0
        joint_constraint.tolerance_above = (np.pi / 2.0)
        joint_constraint.tolerance_below = (np.pi / 2.0)
        joint_constraint.weight = 1.0
        constrained_plan_sub_goal.joint_constraints.append(joint_constraint)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'arm6_joint'
        joint_constraint.position = 0
        joint_constraint.tolerance_above = (np.pi / 2.0)
        joint_constraint.tolerance_below = (np.pi / 2.0)
        joint_constraint.weight = 1.0
        constrained_plan_sub_goal.joint_constraints.append(joint_constraint)

        constrained_plan_client.send_goal(constrained_plan_sub_goal)
        constrained_plan_client.wait_for_result()

        if constrained_plan_client.get_result().result == True:
            remove_switching_pub.publish("on")
            rospy.loginfo("Clearing Octomap")
            clear_octomap()
            rospy.wait_for_service('/body/arm_controller/query_state')
            try:
                query_state = rospy.ServiceProxy('/body/arm_controller/query_state', QueryTrajectoryState)
                resp = query_state(rospy.Time.now())
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            constrained_plan_goal = PlanExecutePoseConstraintsGoal()
            constrained_plan_goal.target_pose = goal.target_pose

            joint_names = resp.name
            joint_positions = resp.position

            joint_constraint = JointConstraint()
            joint_constraint.joint_name = 'arm1_joint'  # joint_names[2]
            joint_constraint.position = joint_positions[2]
            joint_constraint.tolerance_above = (np.pi / 6.0)
            joint_constraint.tolerance_below = (np.pi / 6.0)
            joint_constraint.weight = 1.0
            constrained_plan_goal.joint_constraints.append(joint_constraint)

            joint_constraint = JointConstraint()
            joint_constraint.joint_name = 'arm4_joint'  # joint_names[5]
            joint_constraint.position = joint_positions[5]
            joint_constraint.tolerance_above = (np.pi / 6.0)
            joint_constraint.tolerance_below = (np.pi / 6.0)
            joint_constraint.weight = 1.0
            constrained_plan_goal.joint_constraints.append(joint_constraint)

            joint_constraint = JointConstraint()
            joint_constraint.joint_name = 'arm6_joint'  # joint_names[7]
            joint_constraint.position = joint_positions[7]
            joint_constraint.tolerance_above = (np.pi / 6.0)
            joint_constraint.tolerance_below = (np.pi / 6.0)
            joint_constraint.weight = 1.0
            constrained_plan_goal.joint_constraints.append(joint_constraint)

            constrained_plan_client.send_goal(constrained_plan_goal)
            constrained_plan_client.wait_for_result()

            print constrained_plan_client.get_result()
            if constrained_plan_client.get_result().result == True:
                gripper_control_pub.publish(0.0)    # Gripper close

                rospy.wait_for_service('/body/arm_controller/query_state')
                try:
                    query_state = rospy.ServiceProxy('/body/arm_controller/query_state', QueryTrajectoryState)
                    resp = query_state(rospy.Time.now())
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                goal = FollowJointTrajectoryGoal()
                goal.trajectory.joint_names = list(resp.name)

                point = JointTrajectoryPoint()
                point.positions = list(resp.position)

                point.positions[goal.trajectory.joint_names.index('elevation_joint')] += 0.2
                if point.positions[goal.trajectory.joint_names.index('elevation_joint')] > 0:
                    point.positions[goal.trajectory.joint_names.index('elevation_joint')] = 0
                goal.trajectory.points.append(point)
                point.time_from_start = rospy.Duration(1.0)

                elevation_up_client.send_goal(goal)
                elevation_up_client.wait_for_result()

                rospy.sleep(0.5)

                named_goal = PlanExecuteNamedPoseGoal()
                named_goal.target_name = "grasp_done"

                named_client.send_goal(named_goal)
                named_client.wait_for_result()
                break
            else:   # Control the manipulator back to the home position.
                while named_client.get_result() == False:
                    named_goal = PlanExecuteNamedPoseGoal()
                    named_goal.target_name = "home"

                    named_client.send_goal(named_goal)
                    named_client.wait_for_result()

        print("="*50)

    remove_switching_pub.publish("off")

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    # if len(sys.argv) != 8:
    #     print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z> <r (deg)> <p (deg)> <y (deg)>"
    #     exit(-1)

    main(sys.argv[1:])

    # rospy.spin()