from abc import ABC, abstractmethod
from common.objects import IntrestDescription, TaskConvMsg


class TaskFSM:

    intrest: IntrestDescription

    def __init__(self):
        self.transition_to(DefineTaskIntrest())

    def get_next_message(self, msg: TaskConvMsg):
        return self._state.define_next(msg)

    def transition_to(self, state):
        self._state = state
        self._state.task = self


class State(ABC):
    @property
    def task(self) ->TaskFSM:
        return self._task_fsm

    @task.setter
    def task(self, task_fsm: TaskFSM):
        self._task_fsm = task_fsm

    @abstractmethod
    def define_next(self, msg: TaskConvMsg):
        pass

class DefineTaskIntrest(State):
    INTREST_THRESHOLD = 0.5
    def define_next(self, msg: TaskConvMsg):
        partner_intrest = float(msg.data[0])
        if (partner_intrest > 0.2):
            print(f'received partners intrest: {partner_intrest}')
            self._task_fsm.transition_to(WaitForExec())

class WaitForExec(State):
    def define_next(self, msg: TaskConvMsg):
        print('Waiting for the right time...')

class ExecTask(State):
    def define_next(self, msg: TaskConvMsg):
        pass

class SuperviseTask(State):
    def define_next(self, msg: TaskConvMsg):
        pass