import py_trees
import py_trees_ros
import rospy
import threading
import json

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

class HeadGazeSyncControl(py_trees.behaviour.Behaviour):
    def __init__(self, name="HeadGazeSyncControl", data=True):
        super(HeadGazeSyncControl, self).__init__(name=name)

        self.topic_name = "/set_gaze_sync"
        self.pub_data = data
        self.published = False

    def setup(self, timeout):
        self.publisher = rospy.Publisher(self.topic_name, Bool, queue_size=10)
        rospy.sleep(0.5)
        return True

    def update(self):
        if not self.published:
            self.publisher.publish(self.pub_data)
            rospy.sleep(0.02)
            self.published = True
            return py_trees.common.Status.RUNNING

        rospy.sleep(0.02)
        self.published = False
        return py_trees.common.Status.SUCCESS

class HeadSetGazeTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name="HeadSetGazeTarget", target=PointStamped()):
        super(HeadSetGazeTarget, self).__init__(name=name)

        self.topic_name = "/set_gaze_target"
        self.gaze_target = target
        self.published = False

    def setup(self, timeout):
        self.publisher = rospy.Publisher(self.topic_name, PointStamped, queue_size=10)
        rospy.sleep(0.5)
        return True

    def update(self):
        if not self.published:
            self.publisher.publish(self.gaze_target)
            rospy.sleep(0.02)
            self.published = True
            return py_trees.common.Status.RUNNING

        rospy.sleep(0.02)
        self.published = False
        return py_trees.common.Status.SUCCESS
