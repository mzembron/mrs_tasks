from abc import ABC, abstractmethod
from common.objects import IntrestDescription, TaskConvMsg


class TaskFSM:

    intrest: IntrestDescription

    def __init__(self):
        self._state = DefineCoordIntrest()

    def get_next_message(self, msg: TaskConvMsg):
        self._state.define_next(msg)

    def transition_to(self, state):
        self._state = state
        self._state.task_fsm = self


class State(ABC):
    @property
    def task(self) ->TaskFSM:
        return self._task_fsm

    @task.setter
    def task(self, task_fsm: TaskFSM):
        self.task_fsm = task_fsm

    @abstractmethod
    def define_next(self, msg: TaskConvMsg):
        pass
    
class DefineCoordIntrest(State):
    INTREST_THRESHOLD = 0.5
    def define_next(self, msg: TaskConvMsg):
        partner_intrest = float(msg.data[0])
        if (partner_intrest > 0.2):
            print(f'received partners intrest: {partner_intrest}')

class DefineExecIntrestAsCoord(State):
    def define_next(self, msg: TaskConvMsg):
        pass

class DefineExecintrestAsNonCoord(State):
    def define_next(self, msg: TaskConvMsg):
        pass