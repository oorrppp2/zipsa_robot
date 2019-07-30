import py_trees
import py_trees_ros
import rospy
import threading

from control_msgs.srv import QueryTrajectoryState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class BodyRotateOnly(py_trees_ros.actions.ActionClient):
    def __init__(self, name="RotateBody", target_pose=0.0):
        rospy.wait_for_service('/body/arm_controller/query_state')
        try:
            query_state = rospy.ServiceProxy('/body/arm_controller/query_state', QueryTrajectoryState)
            resp = query_state(rospy.Time.now())
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        joint_names = resp.name
        joint_positions = resp.position

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(resp.name)

        point = JointTrajectoryPoint()
        point.positions = list(resp.position)

        point.positions[goal.trajectory.joint_names.index('body_rotate_joint')] = target_pose
        goal.trajectory.points.append(point)
        point.time_from_start = rospy.Duration(1.0)


        super(BodyRotateOnly, self).__init__(name=name,
            action_spec=FollowJointTrajectoryAction,
            action_goal=goal,
            action_namespace="/body/arm_controller/follow_joint_trajectory",
            override_feedback_message_on_running="moving")

    def initialize(self):
        self.action_goal.trajectory.header.stamp = rospy.Time.now()
        self.action_goal.goal_time_tolerance = rospy.Duration(30.0)



