from enum import Enum


class TaskFSM:
    class State(Enum):
        DEFINE_COORD_INTREST = 1
        COORD_ROLE_ACCEPTED = 2
        DEFINE_EXEC_INTREST = 3
        EXEC_ROLE_ACCEPTED = 4

    def __init__(self):
        self.current_state = TaskFSM.State.DEFINE_COORD_INTREST