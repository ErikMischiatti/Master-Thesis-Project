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

goal_unlock_up= DHBActionGoal()

goal_unlock_lift= DHBActionGoal()
goal_unlock_down= DHBActionGoal()

goal_unlock_up.slowdown_factor = 2
goal_unlock_lift.slowdown_factor = 2
goal_unlock_down.slowdown_factor = 2


goal_unlock_up.goal.position.x = 0.58
goal_unlock_up.goal.position.y = -0.03
goal_unlock_up.goal.position.z = 0.12
goal_unlock_up.goal.orientation.x = 0.99
goal_unlock_up.goal.orientation.y = 0.0
goal_unlock_up.goal.orientation.z = 0.0
goal_unlock_up.goal.orientation.w = -0.01

goal_unlock_down.goal.position.x = 0.58
goal_unlock_down.goal.position.y = -0.05
goal_unlock_down.goal.position.z = 0.082
goal_unlock_down.goal.orientation.x = 0.99
goal_unlock_down.goal.orientation.y = 0.0
goal_unlock_down.goal.orientation.z = 0.0
goal_unlock_down.goal.orientation.w = -0.01

goal_unlock_lift.goal.position.x = 0.58
goal_unlock_lift.goal.position.y = -0.03
goal_unlock_lift.goal.position.z = 0.087
goal_unlock_lift.goal.orientation.x = 0.99
goal_unlock_lift.goal.orientation.y = 0.0
goal_unlock_lift.goal.orientation.z = 0.0
goal_unlock_lift.goal.orientation.w = -0.01

goal_unlock_up.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_unlockclip_up.npz"
goal_unlock_down.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_lower.npz"
goal_unlock_lift.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_raise.npz"


def create_OpenClip_branch():
    pour_branch_seq = py_trees.composites.Sequence("Open Clip", True)


    #ALIGNMENT
    execute_alignment_trajectory = FromConstant(
        name="Alignment",
        action_type=DHBActionAction,
        action_name="DHB_motion",
        action_goal=goal_unlock_up,
        generate_feedback_message=lambda msg: f"Approaching the clip: {msg.feedback.progress:.2f}%"
    )
    

    #UNLOCK
    execute_unlock_trajectory = FromConstant(
        name="Unlock clip",
        action_type=DHBActionAction,
        action_name="DHB_motion",
        action_goal=goal_unlock_lift,
        generate_feedback_message=lambda msg: f"Approaching the clip: {msg.feedback.progress:.2f}%"
    )

    #GRAB CLIP
    execute_unlock_down_trajectory = FromConstant(
        name="Grab clip",
        action_type=DHBActionAction,
        action_name="DHB_motion",
        action_goal=goal_unlock_down,
        generate_feedback_message=lambda msg: f"Approaching the clip: {msg.feedback.progress:.2f}%"
    )

    pour_branch_seq.add_children([
        execute_alignment_trajectory,
        open_gripper(),
        execute_unlock_down_trajectory,
        close_gripper(),
        execute_unlock_trajectory,
        open_gripper(),
        create_GrabClip_branch("open")
    ])

    return pour_branch_seq