import py_trees
import py_trees_ros
import rospy
import threading

from control_msgs.srv import QueryTrajectoryState
from std_msgs.msg import Float64

class MoveJoint(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveJoint", controller_name="/controller", command=0.0):
        super(MoveJoint, self).__init__(name=name)

        self.topic_name = controller_name + "/command"
        self.target_command = command
        self.published = False
        rospy.loginfo( self.topic_name)

    def setup(self, timeout):
        self.publisher = rospy.Publisher(self.topic_name, Float64, queue_size=10)
        rospy.sleep(0.5)
        return True

    def update(self):
        if not self.published:
            self.publisher.publish(self.target_command)
            self.published = True
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS
