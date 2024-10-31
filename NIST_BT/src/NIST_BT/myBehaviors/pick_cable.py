import py_trees
from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myBehaviors.gripper import CheckGripper, open_gripper, close_gripper
from NIST_BT.myBehaviors.placeholders import ActionMocap, Placeholder
from NIST_BT.myBehaviors.move import CreateMoveGoal
from NIST_BT.myPyTrees.action_clients import FromBlackboard, FromConstant
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from dhb_ros.msg import DHBActionAction, DHBActionActionGoal, DHBActionGoal


goal_look = DHBActionGoal()

goal_look.slowdown_factor = 2

goal_look.goal.position.x = 0.4
goal_look.goal.position.y = 0.0
goal_look.goal.position.z = 0.4
goal_look.goal.orientation.x = 1
goal_look.goal.orientation.y = 0
goal_look.goal.orientation.z = 0
goal_look.goal.orientation.w = 0


goal_look.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_look.npz"

def create_pick_branch():
    pour_branch_seq = py_trees.composites.Sequence("Pick up branch", True) 

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

    # LOOK 
    execute_predefined_trajectory = FromConstant(
        name="Look",
        action_type=DHBActionAction,
        action_name="DHB_motion",  
        action_goal=goal_look,  
        generate_feedback_message=lambda msg: f"Eseguendo la traiettoria predefinita: {msg.feedback.progress:.2f}%"
    )

    # cable_pose on Blackboard
    get_cable_pose = ToBlackboard(
        name="Get cable",             
        topic_name="/cable_pose",       
        topic_type=PoseStamped,         
        blackboard_variables={"cable_pose": None}
    )

    # cable_pose from Blackboard
    send_trajectory_goal = FromBlackboard(
        name="Approach Cable",
        action_type= DHBActionAction,  
        action_name="DHB_motion",  
        key="cable_pose_goal",  
        generate_feedback_message=lambda msg: f"Moving to cable: {msg.feedback.progress:.2f}%"
    )


    pour_branch_seq.add_children([
        open_if_closed,
        execute_predefined_trajectory,
        get_cable_pose,
        CreateMoveGoal("Create pick goal", "cable_pose", "cable_pose_goal", "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_pick.npz", 2, -0.02), # with offset in z direction
        send_trajectory_goal,
        close_gripper()
    ])

    return pour_branch_seq