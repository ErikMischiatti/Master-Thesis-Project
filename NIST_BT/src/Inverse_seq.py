import py_trees
from py_trees.behaviours import SetBlackboardVariable, CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression

from NIST_BT.myBehaviors.gripper import CheckGripper
from NIST_BT.myBehaviors.pick_cable import create_pick_branch
from NIST_BT.myBehaviors.remove_from_clip import create_RemoveFromClip_branch
from NIST_BT.myBehaviors.open_clip import create_OpenClip_branch

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

# ps aux | grep Forward_seq.py | grep -v grep | awk '{print $2}' | xargs kill -9 (to kill the BT)


# goal_bottle = condition_net.msg.DmpMoveGoal(
#     create_pose(
#         0.50620868,
#         0.37928719,
#         0.04886917,
#         0.99685603,
#         -0.02599573,
#         0.07204532,
#         -0.02029165,
#     )
# )
# goal_cup = pour_sm.msg.DmpMoveGoal(create_pose(-0.54100161, -0.32174789,  0.32246764, -0.51233398,  0.52622657, -0.48163074, -0.47815408))
# goal_pour = pour_sm.msg.DmpMoveGoal(create_pose(-0.5328075,  -0.3192374,   0.31532626, -0.5192632,   0.52518416, -0.47977704, -0.47366794))
# goal_place = pour_sm.msg.DmpMoveGoal(create_pose(-0.5177281,  -0.09479572,  0.22880241,  0.51327131, -0.4926738,   0.49806452, 0.49573867))
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

root_seq = py_trees.composites.Sequence("inverse", True)


from sensor_msgs.msg import JointState
root_seq.add_children([
    create_OpenClip_branch(),
    # create_pick_branch(),
    # create_RemoveFromClip_branch()
    ]
)

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