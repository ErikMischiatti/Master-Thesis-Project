import py_trees
from NIST_BT.myPyTrees.subscribers import ToBlackboard
from NIST_BT.myBehaviors.gripper import CheckGripper, open_gripper, close_gripper
from NIST_BT.myBehaviors.placeholders import ActionMocap, Placeholder
from NIST_BT.myBehaviors.move import CreateMoveGoal
from NIST_BT.myPyTrees.action_clients import FromConstant
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from dhb_ros.msg import DHBActionAction, DHBActionGoal

goal_put_in_tube = DHBActionGoal()
goal_put_in_tube_2 = DHBActionGoal()
goal_step_back = DHBActionGoal()
goal_push = DHBActionGoal()
goal_flip = DHBActionGoal()

goal_put_in_tube_FAST = DHBActionGoal()
goal_put_in_tube_FAST_2 = DHBActionGoal()
goal_flip_FAST = DHBActionGoal()


goal_put_in_tube.slowdown_factor = 2  
goal_put_in_tube_2.slowdown_factor = 2  
goal_step_back.slowdown_factor = 2  
goal_push.slowdown_factor = 2  
goal_flip.slowdown_factor = 2 

goal_put_in_tube_FAST.slowdown_factor = 2
goal_put_in_tube_FAST_2.slowdown_factor = 2 
goal_flip_FAST.slowdown_factor = 2


# GOALS LONG WAY

goal_put_in_tube.goal.position.x = 0.52
goal_put_in_tube.goal.position.y = 0.15
goal_put_in_tube.goal.position.z = 0.06
goal_put_in_tube.goal.orientation.x = 0.82
goal_put_in_tube.goal.orientation.y = -0.57
goal_put_in_tube.goal.orientation.z = -0.0016
goal_put_in_tube.goal.orientation.w = -0.003


goal_put_in_tube_2.goal.position.x = 0.56
goal_put_in_tube_2.goal.position.y = 0.07
goal_put_in_tube_2.goal.position.z = 0.06
goal_put_in_tube_2.goal.orientation.x = 0.82
goal_put_in_tube_2.goal.orientation.y = -0.57
goal_put_in_tube_2.goal.orientation.z = -0.0016
goal_put_in_tube_2.goal.orientation.w = -0.003


goal_step_back.goal.position.x = 0.52
goal_step_back.goal.position.y = 0.15
goal_step_back.goal.position.z = 0.06
goal_step_back.goal.orientation.x = 0.82
goal_step_back.goal.orientation.y = -0.57
goal_step_back.goal.orientation.z = -0.0016
goal_step_back.goal.orientation.w = -0.003

goal_push.goal.position.x = 0.56
goal_push.goal.position.y = 0.07
goal_push.goal.position.z = 0.06
goal_push.goal.orientation.x = 0.82
goal_push.goal.orientation.y = -0.57
goal_push.goal.orientation.z = -0.0016
goal_push.goal.orientation.w = -0.003


goal_flip.goal.position.x = 0.59
goal_flip.goal.position.y = -0.04
goal_flip.goal.position.z = 0.058
goal_flip.goal.orientation.x = 0.66
goal_flip.goal.orientation.y = 0.74
goal_flip.goal.orientation.z = -0.06
goal_flip.goal.orientation.w = 0.036


# GOALS SHORT WAY

goal_put_in_tube_FAST.goal.position.x = 0.65
goal_put_in_tube_FAST.goal.position.y = -0.05
goal_put_in_tube_FAST.goal.position.z = 0.06
goal_put_in_tube_FAST.goal.orientation.x = 0.97
goal_put_in_tube_FAST.goal.orientation.y = 0.2
goal_put_in_tube_FAST.goal.orientation.z = 0.0
goal_put_in_tube_FAST.goal.orientation.w = 0.02


goal_put_in_tube_FAST_2.goal.position.x = 0.54
goal_put_in_tube_FAST_2.goal.position.y = -0.054
goal_put_in_tube_FAST_2.goal.position.z = 0.06
goal_put_in_tube_FAST_2.goal.orientation.x = 0.97
goal_put_in_tube_FAST_2.goal.orientation.y = 0.2
goal_put_in_tube_FAST_2.goal.orientation.z = 0.0
goal_put_in_tube_FAST_2.goal.orientation.w = 0.02


goal_flip_FAST.goal.position.x = 0.59
goal_flip_FAST.goal.position.y = -0.04
goal_flip_FAST.goal.position.z = 0.06
goal_flip_FAST.goal.orientation.x = 0.97
goal_flip_FAST.goal.orientation.y = 0.2
goal_flip_FAST.goal.orientation.z = 0.0
goal_flip_FAST.goal.orientation.w = 0.02



goal_put_in_tube.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_insert_tube.npz"
goal_put_in_tube_2.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_insert_tube_2.npz"
goal_step_back.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_step_back.npz"
goal_push.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_push.npz"
goal_flip.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_flip.npz"


goal_put_in_tube_FAST.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_insert_tube.npz"
goal_put_in_tube_FAST_2.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_insert_tube_2.npz"
goal_flip_FAST.dhb_file = "/home/asl_team/catkin_ws/src/record_franka_ros/scripts/ours_flip.npz"



def create_PutInTube_branch(approach_type):
    '''
        approach_type (String) :  Two different ways of approaching the tube task, one in a faster way and one in a longer way
    '''
    seq = py_trees.composites.Sequence("Insert in tube", True)

    if approach_type == "long":

        # INSERT (step 1)
        execute_insert_trajectory = FromConstant(
            name="Approach tube",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_put_in_tube,  
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # INSERT  (step 2) 
        execute_insert_2_trajectory = FromConstant(
            name="Insert in tube",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_put_in_tube_2,  
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # STEP BACK    
        execute_back_trajectory_1 = FromConstant(
            name="Step back",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_step_back, 
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # STEP BACK    
        execute_back_trajectory_2 = FromConstant(
            name="Step back",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_step_back, 
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # PUSH   
        execute_push_trajectory_1 = FromConstant(
            name="Push",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_push, 
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # PUSH   
        execute_push_trajectory_2 = FromConstant(
            name="Push",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_push, 
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # APPROACH CABLE - Flip  
        execute_flip_trajectory = FromConstant(
            name="Approach cable",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal= goal_flip, 
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )
        

        seq.add_children([
            execute_insert_trajectory,
            execute_insert_2_trajectory,
            open_gripper(),  
            execute_back_trajectory_1,
            close_gripper(),  
            execute_push_trajectory_1,
            open_gripper(),
            execute_back_trajectory_2,
            close_gripper(),
            execute_push_trajectory_2,
            open_gripper(),
            execute_flip_trajectory,
            close_gripper()
        ])

    else: 
        # INSERT (step 1)
        execute_insert_fast_trajectory = FromConstant(
            name="Approach tube",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_put_in_tube_FAST,  
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # INSERT  (step 2) 
        execute_insert_fast_2_trajectory = FromConstant(
            name="Insert in tube",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal=goal_put_in_tube_FAST_2,  
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )

        # APPROACH CABLE - Flip  
        execute_flip_trajectory = FromConstant(
            name="Approach cable",
            action_type=DHBActionAction,
            action_name="DHB_motion",
            action_goal= goal_flip_FAST, 
            generate_feedback_message=lambda msg: f"Eseguendo la traiettoria: {msg.feedback.progress:.2f}%"
        )


        seq.add_children([
            execute_insert_fast_trajectory,
            execute_insert_fast_2_trajectory
        ])


    return seq
