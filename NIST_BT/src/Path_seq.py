import py_trees
from py_trees.behaviours import SetBlackboardVariable, CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression

from NIST_BT.myBehaviors.gripper import CheckGripper
from NIST_BT.myBehaviors.pick_cable import create_pick_branch
from NIST_BT.myBehaviors.pick_cable_oriented import create_pick_branch_oriented
from NIST_BT.myBehaviors.put_in_tube import create_PutInTube_branch
from NIST_BT.myBehaviors.close_clip import create_CloseClip_branch
from NIST_BT.myBehaviors.alignment import create_Alignment_branch

from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myPyTrees.publishers import FromBlackboard as PublishFromBlackboard
from NIST_BT.myPyTrees.action_clients import FromConstant, FromBlackboard

from NIST_BT.utils import create_pose

import operator
import numpy as np

import rospy
import franka_gripper.msg
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int8

from sensor_msgs.msg import JointState

# ps aux | grep Path_seq.py | grep -v grep | awk '{print $2}' | xargs kill -9 (to kill the BT)

goal_idle = create_pose(
    0.35220399,
    -0.0265472,
    0.42616837,
    1,
    0,
    0,
    0,
)

goal_close = franka_gripper.msg.GraspGoal()
goal_close.epsilon.inner = 0.2
goal_close.epsilon.outer = 0.2
goal_close.force = 20
goal_close.width = 0.0
goal_close.speed = 0.1

goal_open = franka_gripper.msg.MoveGoal()
goal_open.width = 0.08
goal_open.speed = 0.1

rospy.init_node("bt_test")
# py_trees.logging.level = py_trees.logging.Level.DEBUG

print("test")

root_seq = py_trees.composites.Sequence("path", True)

root_seq.add_children([
    create_pick_branch(),
    create_PutInTube_branch("long"),
    create_Alignment_branch(),
    create_PutInTube_branch("fast"),
    # create_pick_branch_oriented()
    # create_PutInTube_branch() 
])


root = root_seq

py_trees.display.render_dot_tree(root)

blackboard = py_trees.blackboard.Client(name="Globals")
blackboard.register_key(key="gripper_closed", access=py_trees.common.Access.WRITE)
blackboard.gripper_closed = 0


def print_tree(tree):
    print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
    # print(py_trees.display.unicode_blackboard())


behaviour_tree = py_trees.trees.BehaviourTree(root=root)
behaviour_tree.setup(timeout=100)

# while not rospy.is_shutdown():
#     behaviour_tree.tick(None, print_tree)
#     input()

try:
    behaviour_tree.tick_tock(
        period_ms=100,
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        pre_tick_handler=None,
        post_tick_handler=print_tree,
    )
except KeyboardInterrupt:
    print("got interrupt")
    behaviour_tree.shutdown()
    exit()

