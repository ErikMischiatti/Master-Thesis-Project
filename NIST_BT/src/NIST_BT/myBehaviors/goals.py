import py_trees
import tf
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import copy

import rospy

def create_pose(p, o):
    msg = Pose()
    msg.position.x = p[0]
    msg.position.y = p[1]
    msg.position.z = p[2]

    msg.orientation.x = o[0]
    msg.orientation.y = o[1]
    msg.orientation.z = o[2]
    msg.orientation.w = o[3]

    res = PoseStamped()
    res.pose = msg

    return res


class TfToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name, source_tf, target_tf, goal_name, offset, oriet):
        """
            relative offset with constant orientations
        """
        super(TfToGoal, self).__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Globals")
        self.blackboard.register_key(key=goal_name, access=py_trees.common.Access.WRITE)
        self._offset = offset
        self._goal_name = goal_name
        self._source_tf = source_tf
        self._target_tf = target_tf
        self._orient = oriet

        self._listener = tf.TransformListener()

    def setup(self):
        self.logger.debug("  %s [TfToGoal::setup()]" % (self.name))

    def initialise(self):
        self.logger.debug("  %s [TfToGoal::initialise()]" % (self.name))

    def update(self):
        try:
            (trans,rot) = self._listener.lookupTransform(self._source_tf, self._target_tf, rospy.Time(0))
            trans = np.array(trans) + self._offset
            rot = self._orient
            goal = create_pose(trans, rot)
        except Exception as e:
            self.feedback_message = str(e)
            return py_trees.common.Status.FAILURE

        setattr(self.blackboard, self._goal_name, goal)
        return py_trees.common.Status.SUCCESS
                

    def terminate(self, new_status):
        self.logger.debug("  %s [TfToGoal::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class CreateRelative(py_trees.behaviour.Behaviour):
    def __init__(self, name, goal_name, offset, oriet):
        """
            relative offset with constant orientations
        """
        super(CreateRelative, self).__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Globals")
        self.blackboard.register_key(key=goal_name, access=py_trees.common.Access.WRITE)
        self._offset = offset
        self._goal_name = goal_name
        self._orient = oriet

        self._listener = tf.TransformListener()

    def setup(self):
        self.logger.debug("  %s [CreateCup::setup()]" % (self.name))

    def initialise(self):
        self.logger.debug("  %s [CreateCup::initialise()]" % (self.name))

    def update(self):
        try:
            (trans,rot) = self._listener.lookupTransform('/fr3_link0', '/fr3_hand_tcp', rospy.Time(0))
            trans = np.array(trans) + self._offset
            print("Trans", trans)
            rot = self._orient
            goal = create_pose(trans, rot)
        except Exception as e:
            self.feedback_message = str(e)
            return py_trees.common.Status.FAILURE

        setattr(self.blackboard, self._goal_name, goal)
        return py_trees.common.Status.SUCCESS
                

    def terminate(self, new_status):
        self.logger.debug("  %s [CreateCup::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class CreateRelativeFixed(py_trees.behaviour.Behaviour):
    def __init__(self, name, goal_name, offset, oriet):
        """
            relative offset with constant orientations
        """
        super(CreateRelativeFixed, self).__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Globals")
        self.blackboard.register_key(key=goal_name, access=py_trees.common.Access.WRITE)
        self._offset = offset
        self._goal_name = goal_name
        self._orient = oriet

        self._listener = tf.TransformListener()

    def setup(self):
        self.logger.debug("  %s [CreateCup::setup()]" % (self.name))

    def initialise(self):
        self.logger.debug("  %s [CreateCup::initialise()]" % (self.name))

    def update(self):
        try:
            (trans,rot) = self._listener.lookupTransform('/fr3_link0', '/fr3_hand_tcp', rospy.Time(0))
            trans = np.array(trans) + self._offset
            trans[2] = 0.2
            rot = self._orient
            goal = create_pose(trans, rot)
        except Exception as e:
            self.feedback_message = str(e)
            return py_trees.common.Status.FAILURE

        setattr(self.blackboard, self._goal_name, goal)
        return py_trees.common.Status.SUCCESS
                

    def terminate(self, new_status):
        self.logger.debug("  %s [CreateCup::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class CreateBook(py_trees.behaviour.Behaviour):
    def __init__(self, name, offset):
        super(CreateBook, self).__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Globals")
        self.blackboard.register_key(key="objects", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="book_goal", access=py_trees.common.Access.WRITE)

        self._offset = offset

    def setup(self):
        self.logger.debug("  %s [CreateBottle::setup()]" % (self.name))

    def initialise(self):
        self.logger.debug("  %s [CreateBottle::initialise()]" % (self.name))

    def update(self):
        p = create_pose([-0.419, 0.290, 0.15], [-0.546, 0.557, -0.431, -0.453])
        self.blackboard.book_goal = p
        return py_trees.common.Status.SUCCESS
        # for p in self.blackboard.objects.poses:
        #     if p.name == "bottle":
        #         pose = copy.deepcopy(p)
        #         pose.pose.position.x += self._offset[0]
        #         pose.pose.position.y += self._offset[1]
        #         pose.pose.position.z += self._offset[2]
        #         self.blackboard.bottle_goal = pose
        #         return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.FAILURE
                

    def terminate(self, new_status):
        self.logger.debug("  %s [CreateBottle::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))