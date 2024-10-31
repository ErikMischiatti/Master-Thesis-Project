import py_trees

from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myPyTrees.action_clients import FromConstant, FromBlackboard

from sensor_msgs.msg import JointState
from py_trees.behaviours import CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression



import operator

import franka_gripper.msg

goal_open = franka_gripper.msg.MoveGoal()
goal_open.width = 0.08
goal_open.speed = 0.1

goal_close = franka_gripper.msg.GraspGoal()
goal_close.epsilon.inner = 0.2
goal_close.epsilon.outer = 0.2
goal_close.force = 20
goal_close.width = 0.0
goal_close.speed = 0.1

class CheckGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        """
            relative offset with constant orientations
        """
        super(CheckGripper, self).__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Globals")
        self.blackboard.register_key(key="griper_msg", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CreateCup::setup()]" % (self.name))

    def initialise(self):
        self.logger.debug("  %s [CreateCup::initialise()]" % (self.name))

    def update(self):
        if self.blackboard.griper_msg.position[0] > 0.07:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE     

    def terminate(self, new_status):
        self.logger.debug("  %s [CreateCup::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

def open_gripper():
    return FromConstant(
                "Open Gripper",
                franka_gripper.msg.MoveAction,
                "franka_gripper/move",
                goal_open)

def close_gripper():
    return FromConstant(
            "Close Gripper",
            franka_gripper.msg.GraspAction,
            "franka_gripper/grasp",
            goal_close,
        )