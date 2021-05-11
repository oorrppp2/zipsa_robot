import py_trees
import py_trees_ros
import rospy
import threading
import actionlib
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, String, Bool, Header, Float64
from moveit_msgs.msg import Constraints
from sensor_msgs.msg import JointState
from behaviors.move_arm_controller import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Print_message(py_trees.behaviour.Behaviour):
    def __init__(self, name="Print_message"):
        super(Print_message, self).__init__(name=name)

    def setup(self, timeout):
        return True

    def update(self):
        print(self.name)
        return py_trees.common.Status.SUCCESS

class Fold_arm(py_trees.behaviour.Behaviour):
    def __init__(self, name="Fold_arm", data=0):
        super(Fold_arm, self).__init__(name=name)
        self.data = data

    def setup(self, timeout):
        return True

    def update(self):
        print(self.name + " : " + str(self.data))
        pub = rospy.Publisher('/body/arm_base_controller/command', Float64, queue_size=10)
        rospy.sleep(0.4)
        pub.publish(data=self.data)
        rospy.sleep(1.0)
        return py_trees.common.Status.SUCCESS

class Publish(py_trees.behaviour.Behaviour):
    def __init__(self, name="Publish", topic_name="", data=""):
        super(Publish, self).__init__(name=name)
        self.data = data
        self.topic_name = topic_name

    def setup(self, timeout):
        return True

    def update(self):
        print(self.name + " : " + str(self.data))
        pub = rospy.Publisher(self.topic_name, String, queue_size=1)
        rospy.sleep(0.4)
        pub.publish(data=self.data)
        rospy.sleep(1.0)
        return py_trees.common.Status.SUCCESS

class Elevation_up(py_trees.behaviour.Behaviour):
#class Elevation_up(py_trees_ros.actions.ActionClient):
	def __init__(self, name="Elevation_up", target_pose=0.0):
		super(Elevation_up, self).__init__(name=name)
		self.target_pose = target_pose
		self.name = name
		self.client = actionlib.SimpleActionClient('/body/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.client.wait_for_server()
#		self.action_goal.trajectory.header.stamp = rospy.Time.now()
#		self.action_goal.goal_time_tolerance = rospy.Duration(30.0)

	def setup(self, timeout):
		return True


	def update(self):
		rospy.wait_for_service('/body/arm_controller/query_state')
		try:
			query_state = rospy.ServiceProxy('/body/arm_controller/query_state', QueryTrajectoryState)
			resp = query_state(rospy.Time.now())
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		joint_names = resp.name
		joint_positions = resp.position

		goal = FollowJointTrajectoryGoal()
		# print(goal)
		goal.trajectory.joint_names = list(resp.name)

		point = JointTrajectoryPoint()
		point.positions = list(resp.position)

#		print(point)
#		print("point.positions[goal.trajectory.joint_names.index('elevation_joint')] : ", point.positions[goal.trajectory.joint_names.index('elevation_joint')] )
		point.positions[goal.trajectory.joint_names.index('elevation_joint')] += self.target_pose
		if point.positions[goal.trajectory.joint_names.index('elevation_joint')] > 0:
			point.positions[goal.trajectory.joint_names.index('elevation_joint')] = 0
		goal.trajectory.points.append(point)
		print(goal)
		point.time_from_start = rospy.Duration(1.0)

		self.client.send_goal(goal)
		self.client.wait_for_result()

		# rospy.sleep(1.0)

		# self.client.send_goal(goal)
		# self.client.wait_for_result()

		print self.client.get_result()

		return py_trees.common.Status.SUCCESS
