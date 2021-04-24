import py_trees
import py_trees_ros
import rospy
import threading
from polly_speech.msg import SpeechAction, SpeechGoal

class Say(py_trees_ros.actions.ActionClient):
    def __init__(self, name="Say", text=""):
        goal  = SpeechGoal(text=text)
        super(Say, self).__init__(name=name, action_spec=SpeechAction, action_goal=goal, action_namespace="/internal_speech",
            override_feedback_message_on_running="saying")