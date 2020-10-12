import py_trees
import py_trees_ros
import rospy
import threading
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, String, Bool, Header, Float64
from moveit_msgs.msg import Constraints

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
