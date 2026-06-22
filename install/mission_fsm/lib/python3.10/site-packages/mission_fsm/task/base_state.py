class BaseState:

    def execute(self, node):
        pass

    def check_transition(self, node):
        return None


# backward compatibility
State = BaseState