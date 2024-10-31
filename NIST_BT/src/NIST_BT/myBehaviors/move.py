import py_trees
from dhb_ros.msg import DHBActionGoal

class CreateMoveGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name, key, goal_name, dhb_path, slowfactor=1, z_offset=0.0):
        """
            relative offset with constant orientations
        """
        super(CreateMoveGoal, self).__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Globals")
        self.blackboard.register_key(
            key=key,
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=goal_name,
            access=py_trees.common.Access.WRITE,
        )

        self.key = key
        self.goal_name = goal_name
        self.slowfactor = slowfactor
        self.z_offset = z_offset  # Offset da applicare all'asse z
        self.dhb_file = dhb_path

    def setup(self):
        self.logger.debug("  %s [CreateMoveGoal::setup()]" % (self.name))

    def initialise(self):
        self.logger.debug("  %s [CreateMoveGoal::initialise()]" % (self.name))

    def update(self):
        try:
            goal = DHBActionGoal()

            # Leggi la posa dal blackboard
            current_pose = getattr(self.blackboard, self.key).pose

            # Applica l'offset lungo l'asse z
            current_pose.position.z += self.z_offset

            # Assegna la posa modificata al goal
            goal.goal = current_pose
            goal.slowdown_factor = self.slowfactor
            goal.dhb_file = self.dhb_file

            # Scrivi il goal modificato sulla blackboard
            setattr(self.blackboard, self.goal_name, goal)
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            print(e)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CreateMoveGoal::terminate()][%s->%s]" % (self.name, self.status, new_status))
