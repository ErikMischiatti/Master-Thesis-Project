import py_trees
from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myBehaviors.gripper import CheckGripper, open_gripper, close_gripper
from NIST_BT.myBehaviors.grab_clip import create_GrabClip_branch
from NIST_BT.myBehaviors.placeholders import ActionMocap, Placeholder
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from NIST_BT.myBehaviors.move import CreateMoveGoal
from NIST_BT.myPyTrees.action_clients import FromBlackboard, FromConstant
from dhb_ros.msg import DHBActionAction, DHBActionActionGoal, DHBActionGoal

goal_look = DHBActionGoal()
goal_look.slowdown_factor = 2

goal_look.goal.position.x = 0.5
goal_look.goal.position.y = 0.0
goal_look.goal.position.z = 0.5
goal_look.goal.orientation.x = 1
goal_look.goal.orientation.y = 0
goal_look.goal.orientation.z = 0
goal_look.goal.orientation.w = 0

goal_look.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_look.npz"

def create_CloseClip_branch():
    pour_branch_seq = py_trees.composites.Sequence("Close Clip", True)


    #  LOOK
    execute_predefined_trajectory = FromConstant(
        name="Look",
        action_type=DHBActionAction,
        action_name="DHB_motion",  
        action_goal=goal_look,  
        generate_feedback_message=lambda msg: f"Eseguendo la traiettoria predefinita: {msg.feedback.progress:.2f}%"
    )


    pour_branch_seq.add_children([
        create_GrabClip_branch("close"),
        # execute_predefined_trajectory,
        # open_gripper()
    ])

    return pour_branch_seq