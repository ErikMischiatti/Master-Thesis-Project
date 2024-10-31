import py_trees
from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myBehaviors.gripper import CheckGripper, open_gripper, close_gripper
from NIST_BT.myBehaviors.placeholders import ActionMocap, Placeholder
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def create_RemoveFromClip_branch():
    seq = py_trees.composites.Sequence("Remove from clip", True)
    seq.add_children([
        ActionMocap("Move", duration=20),
        open_gripper()
    ])
    return seq
    
    