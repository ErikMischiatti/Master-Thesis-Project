import py_trees
from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myBehaviors.gripper import CheckGripper, open_gripper, close_gripper
from NIST_BT.myBehaviors.pick_cable import create_pick_branch
from NIST_BT.myBehaviors.pick_cable_oriented import create_pick_branch_oriented
from NIST_BT.myPyTrees.action_clients import FromConstant
from dhb_ros.msg import DHBActionAction, DHBActionGoal
from NIST_BT.myBehaviors.placeholders import ActionMocap, Placeholder
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

goal_approach_clip = DHBActionGoal()
goal_approach__inverse_clip = DHBActionGoal()
goal_close_clip = DHBActionGoal()
goal_lift_clip = DHBActionGoal()
goal_lower_clip = DHBActionGoal()
goal_open_clip = DHBActionGoal()
goal_approach_cable = DHBActionGoal()

goal_approach_clip.slowdown_factor = 2
goal_approach__inverse_clip.slowdown_factor = 2
goal_close_clip.slowdown_factor = 2
goal_lift_clip.slowdown_factor = 2
goal_lower_clip.slowdown_factor = 2
goal_open_clip.slowdown_factor = 2
goal_approach_cable.slowdown_factor = 2



# GOALS FORWARD
goal_approach_clip.goal.position.x = 0.57
goal_approach_clip.goal.position.y = 0.02
goal_approach_clip.goal.position.z = 0.06
goal_approach_clip.goal.orientation.x = 0.72
goal_approach_clip.goal.orientation.y = -0.69
goal_approach_clip.goal.orientation.z = -0.05
goal_approach_clip.goal.orientation.w = 0.002

goal_close_clip.goal.position.x = 0.58
goal_close_clip.goal.position.y = -0.03
goal_close_clip.goal.position.z = 0.095
goal_close_clip.goal.orientation.x = 0.71
goal_close_clip.goal.orientation.y = -0.71
goal_close_clip.goal.orientation.z = 0.0
goal_close_clip.goal.orientation.w = 0.0

goal_lift_clip.goal.position.x = 0.58
goal_lift_clip.goal.position.y = -0.03
goal_lift_clip.goal.position.z = 0.16
goal_lift_clip.goal.orientation.x = 0.71
goal_lift_clip.goal.orientation.y = -0.71
goal_lift_clip.goal.orientation.z = 0.0
goal_lift_clip.goal.orientation.w = 0.0

goal_lower_clip.goal.position.x = 0.58
goal_lower_clip.goal.position.y = -0.03
goal_lower_clip.goal.position.z = 0.08
goal_lower_clip.goal.orientation.x = 0.71
goal_lower_clip.goal.orientation.y = -0.71
goal_lower_clip.goal.orientation.z = 0.0
goal_lower_clip.goal.orientation.w = 0.0

# GOALS INVERSE
goal_approach__inverse_clip.goal.position.x = 0.58
goal_approach__inverse_clip.goal.position.y = -0.06
goal_approach__inverse_clip.goal.position.z = 0.09
goal_approach__inverse_clip.goal.orientation.x = 0.71
goal_approach__inverse_clip.goal.orientation.y = -0.71
goal_approach__inverse_clip.goal.orientation.z = 0.0
goal_approach__inverse_clip.goal.orientation.w = 0.0

goal_open_clip.goal.position.x = 0.64
goal_open_clip.goal.position.y = -0.042
goal_open_clip.goal.position.z = 0.12
goal_open_clip.goal.orientation.x = 0.99
goal_open_clip.goal.orientation.y = 0.0
goal_open_clip.goal.orientation.z = 0.0
goal_open_clip.goal.orientation.w = -0.01

goal_approach_cable.goal.position.x = 0.64
goal_approach_cable.goal.position.y = -0.041
goal_approach_cable.goal.position.z = 0.06
goal_approach_cable.goal.orientation.x = 0.99
goal_approach_cable.goal.orientation.y = 0.0
goal_approach_cable.goal.orientation.z = 0.0
goal_approach_cable.goal.orientation.w = -0.01


goal_approach_clip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_grabclip.npz"
goal_close_clip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_closeclip.npz"
goal_lift_clip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_raise.npz"
goal_lower_clip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_lower.npz"

goal_approach__inverse_clip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_lower.npz"
goal_open_clip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_raise.npz"
goal_approach_cable.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_approach_cable.npz"

def create_GrabClip_branch(grasp_type):
    '''
        grasp_type (String) :  Two different ways of grabbing, one for closing one for opening
    '''
    pour_branch_seq = py_trees.composites.Sequence("Grab Clip", True)

    open_if_closed = py_trees.composites.Sequence("Open if closed", True)

    if_closed = py_trees.composites.Selector("Closed", True)
    if_closed.add_children([
        CheckGripper("check open"),
        open_gripper()
    ])

    open_if_closed.add_children([
        ToBlackboard(
            "Get gripper",
            "/franka_gripper/joint_states",
            JointState,
            {"griper_msg": None},
        ),
        if_closed
    ])

    if grasp_type == "close":

        # APPROACHING 
        execute_approach_trajectory = FromConstant(
            name="Grab Clip",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_approach_clip,
            generate_feedback_message=lambda msg: f"Approaching the clip: {msg.feedback.progress:.2f}%"
        )

        # CLOSING 
        execute_close_trajectory = FromConstant(
            name="Close Clip",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_close_clip,
            generate_feedback_message=lambda msg: f"Closing the clip: {msg.feedback.progress:.2f}%"
        )

        # LOCKING - LIFT 
        execute_lift_trajectory = FromConstant(
            name="Lock Clip (up)",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_lift_clip,
            generate_feedback_message=lambda msg: f"Locking the clip (first step): {msg.feedback.progress:.2f}%"
        )

        # LOCKING - LOWER
        execute_lower_trajectory = FromConstant(
            name="Lock Clip (down)",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_lower_clip,
            generate_feedback_message=lambda msg: f"Locking the clip (second step): {msg.feedback.progress:.2f}%"
        )

        pour_branch_seq.add_children([
            open_if_closed,
            execute_approach_trajectory,
            close_gripper(), 
            execute_close_trajectory,
            open_gripper(),
            execute_lift_trajectory,
            close_gripper(),
            execute_lower_trajectory 
        ])
    

    else:

        # OPENING CLIP - LIFTING THE CABLE 
        execute_open_trajectory = FromConstant(
            name="Open clip",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_open_clip,
            generate_feedback_message=lambda msg: f"Approaching the clip: {msg.feedback.progress:.2f}%"
        )


        pour_branch_seq.add_children([
            create_pick_branch_oriented(),
            execute_open_trajectory,
            open_gripper() 
        ])

    return pour_branch_seq
