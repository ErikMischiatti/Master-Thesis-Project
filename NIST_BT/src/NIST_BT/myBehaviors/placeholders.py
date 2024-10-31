import py_trees

class Placeholder(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Placeholder, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [Placeholder::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Placeholder::initialise()]" % self.name)

    def update(self):
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [Placeholder::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class ActionMocap(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration):
        super(ActionMocap, self).__init__(name)

        self.dur = duration
        self._i = 0
        self.sent_goal = False

    def setup(self):
        self.logger.debug("  %s [Placeholder::setup()]" % self.name)

    def initialise(self):
        print("Initilaizing", self.name)
        self.logger.debug("  %s [Placeholder::initialise()]" % self.name)

        self._i = 0
        self.sent_goal = False

    def update(self):
        print("calling update")
        # import pdb
        # pdb.set_trace()
        if not self.sent_goal:
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            s = py_trees.common.Status.RUNNING
        
        if self._i < self.dur:
            self._i += 1
            self.feedback_message = "executing action"
            s = py_trees.common.Status.RUNNING
        else:
            self.feedback_message = "Finished action"
            s = py_trees.common.Status.SUCCESS
        
        print(s)
        return s

    def terminate(self, new_status):
        print("Terminating action", self.name, self.status, new_status)
        self.logger.debug(
            "  %s [Placeholder::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )
