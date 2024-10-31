import py_trees
from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myBehaviors.gripper import CheckGripper, open_gripper, close_gripper
from NIST_BT.myBehaviors.placeholders import ActionMocap, Placeholder
from NIST_BT.myBehaviors.move import CreateMoveGoal
from NIST_BT.myPyTrees.action_clients import FromConstant
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from dhb_ros.msg import DHBActionAction, DHBActionGoal

goal_alignment_up = DHBActionGoal()

goal_alignment_up.slowdown_factor = 2

goal_alignment_up.goal.position.x = 0.6
goal_alignment_up.goal.position.y = -0.08
goal_alignment_up.goal.position.z = 0.21
goal_alignment_up.goal.orientation.x = 0.97
goal_alignment_up.goal.orientation.y = 0.2
goal_alignment_up.goal.orientation.z = 0.0
goal_alignment_up.goal.orientation.w = 0.02


goal_alignment_up.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_alignment.npz"

def create_Alignment_branch():
    seq = py_trees.composites.Sequence("Alignment with the second tube", True)

    #ALIGNMENT
    execute_alignment_trajectory = FromConstant(
        name="Alignment",
        action_type=DHBActionAction,
        action_name="DHB_motion",
        action_goal=goal_alignment_up,
        generate_feedback_message=lambda msg: f"Approaching the clip: {msg.feedback.progress:.2f}%"
    )
    

    seq.add_children([
        execute_alignment_trajectory
    ])
    
    return seq