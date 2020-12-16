import actionlib
import py_trees
import rospy
import math
import actionlib_msgs.msg as actionlib_msgs
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from moveit_msgs.msg import JointConstraint


class OrderActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, action_namespace="/action",
                 override_feedback_message_on_running="moving"):
        super(OrderActionClient, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running

        self.blackboard = py_trees.blackboard.Blackboard()
        # self.blackboard.qrcode_pose = PoseStamped()
        self.blackboard.target = ""

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
            self.blackboard.target = result.code_data
            print("Yes, I will find <"+self.blackboard.target+">")
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


class QRCodeActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, action_namespace="/action",
                 override_feedback_message_on_running="moving"):
        super(QRCodeActionClient, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.qrcode_pose = PoseStamped()
        self.blackboard.frame_id = ""

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

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            # self.action_goal.target = self.blackboard.target
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
            self.blackboard.qrcode_pose = result.pose
            self.blackboard.frame_id = result.code_data
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


class GraspActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action Client", action_spec=None, action_goal=None, x_offset=0.0, y_offset=0.0, z_offset=0.0, action_namespace="/action",
                 override_feedback_message_on_running="moving", constraint=False, joint=""):
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

    def update(self):
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            # if self.blackboard.frame_id != self.blackboard.target:
            #     print("Detected fail!")
            #     return py_trees.Status.SUCCESS

            #update goal data from blackboard
            self.action_goal.target_pose.header.frame_id = "base_footprint"
            self.action_goal.target_pose.pose.position.x = self.blackboard.qrcode_pose.pose.position.x# + self.x_offset
            self.action_goal.target_pose.pose.position.y = self.blackboard.qrcode_pose.pose.position.y# + self.y_offset
            self.action_goal.target_pose.pose.position.z = self.blackboard.qrcode_pose.pose.position.z# + self.z_offset
            #self.action_goal.target_pose.pose.position.x = self.x_offset
            #self.action_goal.target_pose.pose.position.y = self.y_offset
            #self.action_goal.target_pose.pose.position.z = self.z_offset

            # rospy.loginfo(self.action_goal.target_pose.pose.position.x)

            theta = math.atan2(self.action_goal.target_pose.pose.position.y, self.action_goal.target_pose.pose.position.x)

            quat = quaternion_from_euler(0.0, 0.0, theta)

            self.action_goal.target_pose.pose.orientation.x = quat[0]
            self.action_goal.target_pose.pose.orientation.y = quat[1]
            self.action_goal.target_pose.pose.orientation.z = quat[2]
            self.action_goal.target_pose.pose.orientation.w = quat[3]
#            self.action_goal.target_pose.pose.orientation.x = 0
#            self.action_goal.target_pose.pose.orientation.y = 0
#            self.action_goal.target_pose.pose.orientation.z = 0
#            self.action_goal.target_pose.pose.orientation.w = 1.0

            if self.constraint:
                if self.joint == "":
                    print("Constraint joint not defined.")
                    return py_trees.Status.FAILURE

                joint_constraint = JointConstraint()
                joint_constraint.joint_name = self.joint
                joint_constraint.position = 0.0
                joint_constraint.tolerance_above = 30 * math.pi / 180.0
                joint_constraint.tolerance_below = 30 * math.pi / 180.0
                joint_constraint.weight = 1.0

		# arm6_joint makes singularity, so always constraint the arm6_joint.
		joint_constraint_arm6_joint = JointConstraint()
		joint_constraint_arm6_joint.joint_name = "arm6_joint"
		joint_constraint_arm6_joint.position = float(0.0)
		joint_constraint_arm6_joint.tolerance_above = float(10) * math.pi / 180.0
		joint_constraint_arm6_joint.tolerance_below = float(10) * math.pi / 180.0
		joint_constraint_arm6_joint.weight = 1.0
                self.action_goal.joint_constraints.append(joint_constraint)
                self.action_goal.joint_constraints.append(joint_constraint_arm6_joint)
                print("Joint constrained.")

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
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()

        if result:
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

