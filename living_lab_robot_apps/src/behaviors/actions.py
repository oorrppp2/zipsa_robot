#-*- encoding: utf8 -*-
import actionlib
import py_trees
import rospy
import math
import actionlib_msgs.msg as actionlib_msgs
from std_msgs.msg import Empty, String, Bool, Header
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from moveit_msgs.msg import JointConstraint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from behaviors.speech import *
from polly_speech.msg import SpeechAction, SpeechGoal
from control_msgs.srv import QueryTrajectoryState


class OrderActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, action_namespace="/action",
                 override_feedback_message_on_running="moving"):
        super(OrderActionClient, self).__init__(name)
        self.saying_action_client = None
        self.saying_action_namespace = "/internal_speech"
        self.saying_action_spec = SpeechAction
        self.saying_action_goal = None
        self.action_client = None
        self.sent_goal = False
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running

        self.blackboard = py_trees.blackboard.Blackboard()
        # self.blackboard.object_pose = PoseStamped()
        self.blackboard.target = ""
        self.done_scene_publisher = rospy.Publisher("/wait_done_scene", String, queue_size=10)

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the rotate action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False


        self.saying_action_client = actionlib.SimpleActionClient(
            self.saying_action_namespace,
            self.saying_action_spec
        )
        if not self.saying_action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the rotate action server at '{1}'".format(self.__class__.__name__, self.saying_action_namespace))
            self.saying_action_client = None
            return False
        return True

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()

        if result:
            self.blackboard.target = result.data
            print(result)
            next_state = None
            if result.data == "go_home":
                self.saying_action_goal = SpeechGoal(text='네, 집으로 돌아가겠습니다.')
                next_state = 'scene_9_done'   # go home.
            else:
                print("Yes, I will find <"+self.blackboard.target+">")
                if result.data == 'cup':
                    self.saying_action_goal = SpeechGoal(text='네, 컵을 가져다 드리겠습니다.')
                elif result.data == 'bottle':
                    self.saying_action_goal = SpeechGoal(text='네, tumbler를 가져다 드리겠습니다.')
                elif result.data == 'milk':
                    self.saying_action_goal = SpeechGoal(text='네, 우유를 가져다 드리겠습니다.')
                next_state = 'scene_3_done'   # Move to shelf

            self.saying_action_client.send_goal(self.saying_action_goal)
            print("Publish : " + next_state)
            self.done_scene_publisher.publish(next_state)   # go home.
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False


class ObjectDetectionActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, action_namespace="/action",
                 override_feedback_message_on_running="moving"):
        super(ObjectDetectionActionClient, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.object_pose = PoseStamped()
        self.blackboard.frame_id = ""

	self.n_try = 20

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.request_publisher = rospy.Publisher("/request_detection", String, queue_size=10)
        self.done_publisher = rospy.Publisher("/request_detection", String, queue_size=10)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the rotate action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

    def update(self):
        self.request_publisher.publish('request')
	# print(self.feedback_message)
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_goal.target = self.blackboard.target
#            self.action_goal = "Test"
            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING

        self.feedback_message = self.action_client.get_goal_status_text()
        result = self.action_client.get_result()

        if result:
            self.blackboard.object_pose = result.pose
            self.blackboard.frame_id = result.data
            print("Conversion succeed!!")
            print("Result with (robot coordinate)")
            print(result)
            self.done_publisher.publish('done')
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False


class GraspActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, x_offset=0.0, y_offset=0.0, z_offset=0.0, action_namespace="/action",
                 override_feedback_message_on_running="moving", constraint=False, joint=[], mode=""):
        super(GraspActionClient, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.constraint = constraint
        self.joint = joint
        self.mode = mode

        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the rotate action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        self.fail_count = 0	# Try 10 times.

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            if self.blackboard.frame_id != self.blackboard.target:
                print("Detected fail!")
                return py_trees.Status.FAILURE

            #update goal data from blackboard
            self.action_goal.target_pose.header.frame_id = "base_footprint"
#            self.action_goal.target_pose.pose.position.x = self.blackboard.object_pose.pose.position.x
#            self.action_goal.target_pose.pose.position.y = self.blackboard.object_pose.pose.position.y
#            self.action_goal.target_pose.pose.position.z = self.blackboard.object_pose.pose.position.z
            self.action_goal.target_pose.pose.position.x = self.blackboard.object_pose.pose.position.x + self.x_offset
            self.action_goal.target_pose.pose.position.y = self.blackboard.object_pose.pose.position.y + self.y_offset
            self.action_goal.target_pose.pose.position.z = self.blackboard.object_pose.pose.position.z + self.z_offset

            if self.blackboard.target == "bottle":
                self.action_goal.target_pose.pose.position.z -= 0.03
            elif self.blackboard.target == "milk":
                self.action_goal.target_pose.pose.position.z -= 0.06
            if self.mode == "put":
#	            self.action_goal.target_pose.pose.position.x = self.blackboard.object_pose.pose.position.x
#	            self.action_goal.target_pose.pose.position.z = self.blackboard.object_pose.pose.position.z + 0.1
#	            self.action_goal.target_pose.pose.position.y = -self.blackboard.object_pose.pose.position.y
	            self.action_goal.target_pose.pose.position.x = 0.85
	            self.action_goal.target_pose.pose.position.z = 0.55
	            self.action_goal.target_pose.pose.position.y = 0.0

            theta = math.atan2(self.action_goal.target_pose.pose.position.y, self.action_goal.target_pose.pose.position.x)

            quat = quaternion_from_euler(0.0, 0.0, theta)

            self.action_goal.target_pose.pose.orientation.x = quat[0]
            self.action_goal.target_pose.pose.orientation.y = quat[1]
            self.action_goal.target_pose.pose.orientation.z = quat[2]
            self.action_goal.target_pose.pose.orientation.w = quat[3]

            if self.constraint:
                if len(self.joint) == 0:
                    print("Constraint joint not defined.")
                    return py_trees.Status.FAILURE
                for joint in self.joint:
                    joint_constraint = JointConstraint()
                    joint_constraint.joint_name = joint
                    joint_constraint.position = self.joint[joint_constraint.joint_name][0]
                    joint_constraint.tolerance_above = self.joint[joint_constraint.joint_name][1]
                    joint_constraint.tolerance_below = self.joint[joint_constraint.joint_name][2]
                    joint_constraint.weight = 1.0
                    self.action_goal.joint_constraints.append(joint_constraint)

                    print("Joint constrained : " + str(joint_constraint))

            # print("<== action goal ==>")
            # print("Find : " + self.blackboard.frame_id)
            # print("Goal position : (" + str(self.action_goal.target_pose.pose.position.x)
            #       + " , " + str(self.action_goal.target_pose.pose.position.y)
            #       + " , " + str(self.action_goal.target_pose.pose.position.z) + ") away from /base_footprint")
            print("Goal position : (" + str(self.action_goal.target_pose.pose.position.x)
                  + " , " + str(self.action_goal.target_pose.pose.position.y)
                  + " , " + str(self.action_goal.target_pose.pose.position.z) + ")")
            print("Goal orientation : ", quat)

            self.action_client.send_goal(self.action_goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING

#        print("self.action_client.get_state() : " + str(self.action_client.get_state()))
#        print("self.feedback_message : " + str(self.feedback_message))
        self.feedback_message = self.action_client.get_goal_status_text()

        # Failure case
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
#            if self.fail_count < 5:
#                self.sent_goal = False
#                self.fail_count += 1
#                print("Action failed. Retry :: " + str(self.fail_count) + " try")
#                return py_trees.Status.RUNNING
#            else:
#                print("Tried ",self.fail_count, " times. Action failed.")
#                self.fail_count = 0
#                return py_trees.Status.FAILURE
            return py_trees.Status.FAILURE

        result = self.action_client.get_result()

        if result:
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        # print("terminate")
        # print("new_status : " + str(new_status))
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            print("motion_state : " + str(motion_state))
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False


class Body_Rotate(py_trees.behaviour.Behaviour):
    def __init__(self, name="Body_rotation"):
        super(Body_Rotate, self).__init__(name=name)
        self.name = name
        self.client = actionlib.SimpleActionClient('/body/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
        self.blackboard = py_trees.blackboard.Blackboard()
        # print("\n", self.name, "\n")

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
        print(goal)
        goal.trajectory.joint_names = list(resp.name)

        point = JointTrajectoryPoint()
        point.positions = list(resp.position)

        theta = math.atan2(self.blackboard.object_pose.pose.position.y, self.blackboard.object_pose.pose.position.x)
        point.positions[goal.trajectory.joint_names.index('body_rotate_joint')] = theta
		# if point.positions[goal.trajectory.joint_names.index('elevation_joint')] > 0:
		# 	point.positions[goal.trajectory.joint_names.index('elevation_joint')] = 0
        goal.trajectory.points.append(point)
        print(goal)
        point.time_from_start = rospy.Duration(1.0)

        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.sleep(1.0)

        self.client.send_goal(goal)
        self.client.wait_for_result()

        print self.client.get_result()

        return py_trees.common.Status.SUCCESS
