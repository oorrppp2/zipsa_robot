import py_trees
import py_trees_ros
import rospy
import threading

from control_msgs.srv import QueryTrajectoryState
from std_msgs.msg import String

class DonePlayScene(py_trees.behaviour.Behaviour):
    def __init__(self, name="DonePlayScene"):
        super(DonePlayScene, self).__init__(name=name)

        self.topic_name = "wait_done_scene"
        self.published = False

    def setup(self, timeout):
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        rospy.sleep(0.5)
        return True

    def update(self):
        if not self.published:
            self.publisher.publish("")
            self.published = True
            return py_trees.common.Status.RUNNING

        self.published = False
        return py_trees.common.Status.SUCCESS
