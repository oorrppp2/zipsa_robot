import py_trees
import py_trees_ros
import rospy
import threading
import json

from std_msgs.msg import String

class LampControl(py_trees.behaviour.Behaviour):
    def __init__(self, name="LampControl", mode=0, args="{}"):
        super(LampControl, self).__init__(name=name)

        self.topic_name = "/set_lamp_mode"
        self.lamp_args = args
        self.lamp_mode = mode
        self.published = False

    def setup(self, timeout):
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        rospy.sleep(0.5)
        return True

    def update(self):
        if not self.published:
            json_data = {"mode": self.lamp_mode, "args": self.lamp_args}
            self.publisher.publish(json.dumps(json_data))
            self.published = True
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS