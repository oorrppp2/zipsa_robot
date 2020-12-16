import py_trees
import py_trees_ros
import rospy
import threading
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, String, Bool, Header, Float64
from moveit_msgs.msg import Constraints
from sensor_msgs.msg import JointState

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
	def __init__(self, name="Elevation_up", topic_name="", data=""):
		super(Elevation_up, self).__init__(name=name)
		self.data = data
		self.topic_name = topic_name
		self.elevation_position = 0.0

	def callback(self, msg):
		self.elevation_position
		self.elevation_position = msg.position[-2]
	#    print msg.position[-2]
		rospy.sleep(1.0)

	def setup(self, timeout):
		self.joint_sub = rospy.Subscriber('/body/joint_states', JointState, self.callback)
		return True

	def update(self):
		print(self.elevation_position)
		return py_trees.common.Status.SUCCESS
