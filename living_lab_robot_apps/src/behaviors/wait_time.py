import py_trees
import py_trees_ros
import rospy
import threading


class WaitForTime(py_trees.behaviour.Behaviour):
    def __init__(self, name="WaitForTime", time=0.5):
        super(WaitForTime, self).__init__(name=name)
        self.is_started = False
        self.duration_time = time

    def setup(self, timeout):
        self.is_started = False
        return True

    def update(self):
        if not self.is_started:
            self.start_time = rospy.Time.now()
            self.end_time = self.start_time + rospy.Duration(self.duration_time)
            self.is_started = True
        else:
            if rospy.Time.now() < self.end_time:
                return py_trees.common.Status.RUNNING
            else:
                self.is_started = False
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING