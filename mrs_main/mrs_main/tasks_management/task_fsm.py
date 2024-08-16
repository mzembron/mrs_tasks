from abc import ABC, abstractmethod
from common.objects import IntrestDescription, TaskConvMsg
from common.conversation_data import MrsConvPerform
from common.exceptions import InvalidMsgPerformative

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
    def task(self) -> TaskFSM:
        return self._task_fsm

    @task.setter
    def task(self, task_fsm: TaskFSM):
        self._task_fsm = task_fsm

    def define_next(self, msg: TaskConvMsg) -> TaskConvMsg:
        """ Routing method, runs method specific to communicate type,
            thanks to that return adequate response or no response at all. """
        if (msg.performative == MrsConvPerform.declare_coord_intrest): 
            return self.respond_to_coord_intrest_declaration(msg)
        elif (msg.performative == MrsConvPerform.declare_ex_intrest): 
            return self.respond_to_exec_intrest_declaration(msg)
        elif (msg.performative == MrsConvPerform.propose_exec_role):
            return self.respond_to_exec_proposal(msg)
        elif (msg.performative == MrsConvPerform.accept_exec_proposal):
            return self.respond_to_exec_acceptance(msg)
        else:
            raise InvalidMsgPerformative
        
    def respond_to_coord_intrest_declaration(self, msg: TaskConvMsg):
        return
    
    def respond_to_exec_intrest_declaration(self, msg: TaskConvMsg):
        return
    
    def respond_to_exec_proposal(self, msg: TaskConvMsg):
        return
    
    def respond_to_exec_acceptance(self, msg: TaskConvMsg):
        return

class DefineTaskIntrest(State):
    INTREST_THRESHOLD = 0.5
    def respond_to_coord_intrest_declaration(self, msg: TaskConvMsg) -> TaskConvMsg:
        partner_intrest = float(msg.data[0])
        reply_msg = TaskConvMsg() 
        if (partner_intrest > self.INTREST_THRESHOLD):
            print(f'received partners intrest: {partner_intrest}')
            reply_msg.performative = MrsConvPerform.propose_exec_role
            reply_msg.data = [msg.sender]
            print(f'Proposing coord role to: {msg.sender}')
            self._task_fsm.transition_to(WaitForExec())
        else:
            reply_msg.performative = MrsConvPerform.declare_coord_intrest
            temp_coord_intrest = '0.4' #TODO: remove coord intrest at all, 
                                            # every agent should take part in supervising
            reply_msg.data = [temp_coord_intrest]
        reply_msg.short_id = msg.short_id
        return reply_msg

class WaitForExec(State):
    pass

class ExecTask(State):
    pass

class SuperviseTask(State):
    pass