import actionlib
import franka_gripper.msg

class Gripper:

    def __init__(self):
        # Get gripper control action
        self._open_client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        self._close_client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        print("wating for action server")
        self._open_client.wait_for_server()
        self._close_client.wait_for_server()

    def open(self):
        print("Oppening")
        goal = franka_gripper.msg.MoveGoal()

        goal.width = 0.08
        goal.speed = 0.5
        self._open_client.send_goal(goal)


    def close(self):
        print("Closing")
        goal = franka_gripper.msg.GraspGoal()

        goal.width = 0.0
        goal.speed = 0.5
        goal.force = 60
        goal.epsilon.inner = 0.2
        goal.epsilon.outer = 0.2
        self._close_client.send_goal(goal)

