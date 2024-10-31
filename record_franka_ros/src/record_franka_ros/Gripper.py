import actionlib
import franka_gripper.msg

class Gripper:
    _client_graps = None
    def __init__(self,
                 action_grasp='/franka_gripper/grasp'):
        
        # Get gripper control action
        if Gripper._client_graps is None:
            Gripper._client_graps = actionlib.SimpleActionClient(action_grasp, franka_gripper.msg.GraspAction)

            Gripper._client_graps.wait_for_server()

        print("init")

    def open(self):
        goal = franka_gripper.msg.GraspGoal()
        goal.epsilon.inner = 0.2
        goal.epsilon.outer = 0.2
        goal.force = 0.1
        goal.width = 0.08
        goal.speed = 0.1
    
        # Sends the goal to the action server.
        Gripper._client_graps.send_goal(goal)
        # Gripper._client_graps.wait_for_result()
        print("opened")

    def close(self):
        goal = franka_gripper.msg.GraspGoal()
        goal.epsilon.inner = 0.2
        goal.epsilon.outer = 0.2
        goal.force = 60.0
        goal.width = 0.0
        goal.speed = 0.1
    
        # Sends the goal to the action server.
        Gripper._client_graps.send_goal(goal)
        # Gripper._client_graps.wait_for_result()
        print("closed")

    def close_client(self):
        Gripper._client_graps.cancel_all_goals()
