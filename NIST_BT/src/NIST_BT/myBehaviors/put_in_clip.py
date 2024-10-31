import py_trees
from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myBehaviors.gripper import CheckGripper, open_gripper, close_gripper
from NIST_BT.myBehaviors.placeholders import ActionMocap, Placeholder
from NIST_BT.myBehaviors.move import CreateMoveGoal
from NIST_BT.myPyTrees.action_clients import FromConstant
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from dhb_ros.msg import DHBActionAction, DHBActionGoal


goal_put_in_clip = DHBActionGoal()
goal_put_in_clip.slowdown_factor = 2  


goal_put_in_clip.goal.position.x = 0.64
goal_put_in_clip.goal.position.y = -0.041
goal_put_in_clip.goal.position.z = 0.058
goal_put_in_clip.goal.orientation.x = 0.99
goal_put_in_clip.goal.orientation.y = 0.0
goal_put_in_clip.goal.orientation.z = 0.0
goal_put_in_clip.goal.orientation.w = 0.01

goal_put_in_clip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_insert.npz"

def create_PutInClip_branch():
    seq = py_trees.composites.Sequence("Insert in clip", True)

    execute_predefined_trajectory = FromConstant(
        name="Insert in clip",
        action_type=DHBActionAction,
        action_name="DHB_motion",
        action_goal=goal_put_in_clip,  
        generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
    )

    seq.add_children([
        execute_predefined_trajectory,
        open_gripper()  
    ])
    return seq
