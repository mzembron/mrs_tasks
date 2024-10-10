from abc import ABC, abstractmethod
from typing import Callable, Any

from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.common.conversation_data import MrsConvPerform
from mrs_main.common.exceptions import InvalidMsgPerformative
from mrs_main.tasks_management.dependency_manager import TaskDependencyManager
from mrs_main.task_execution.task_executor import TaskExecutor
from mrs_main.task_execution.concrete_executors.dummy_executor import DummyExecutor

class TaskFSM:


    def __init__(self, dependency_manager: TaskDependencyManager, 
                    task_desc: dict,
                    interest_desc: IntrestDescription,
                    task_finished_callback: Callable[..., Any],
                    concrete_executor=DummyExecutor) -> None:
        self.transition_to(DefineTaskIntrest())
        self._dependency_manager = dependency_manager
        self._executor = TaskExecutor(task_desc, self.on_task_finished, concrete_executor)
        self.interest_desc = interest_desc
        self.task_finished_callback = task_finished_callback

    def get_next_message(self, msg: TaskConvMsg):
        return self._state.define_next(msg)

    def transition_to(self, state):
        self._state = state
        self._state.task_fsm = self
        self._state.change_state_routine()
        
    def inform_about_finished_dependency(self):
        """ Notify the dependency manager that this task is complete, allowing it
            to resolve dependencies for other tasks dependent on this one. """
        self._dependency_manager.notify_on_finish()
    
    def resume_after_finished_dependencies(self) -> None:
        """ Resume task (move to task-execution state) after all dependencies are resolved """
        self._state.continue_after_resolved_dependencies()
    
    def start_execution(self) -> None:
        self._executor.start_supervising_execution()

    def on_task_finished(self):
        self._state.on_task_finished()

class State(ABC):
    @property
    def task(self) -> TaskFSM:
        return self._task_fsm

    @task.setter
    def task_fsm(self, task_fsm: TaskFSM):
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
        elif (msg.performative == MrsConvPerform.request_exec_info):
            return self.respond_to_exec_info_request(msg)
        else:
            raise InvalidMsgPerformative

    def change_state_routine(self):
        """ transition method, allows for state specific behavior on transition """    
        pass

# virtual methods
    def continue_after_resolved_dependencies(self):
        return
    
    def on_task_finished(self):
        assert False, "[ DEBUG LOG ] Oops! Task should not be finished in that state!"
# virtual methods - respond to specific msg content
    def respond_to_coord_intrest_declaration(self, msg: TaskConvMsg):
        return
    
    def respond_to_exec_intrest_declaration(self, msg: TaskConvMsg):
        return
    
    def respond_to_exec_proposal(self, msg: TaskConvMsg):
        return
    
    def respond_to_exec_acceptance(self, msg: TaskConvMsg):
        return
    
    def respond_to_exec_info_request(self, msg: TaskConvMsg):
        return

class DefineTaskIntrest(State):
    INTREST_THRESHOLD = 0.5
    def respond_to_coord_intrest_declaration(self, msg: TaskConvMsg) -> TaskConvMsg:
        partner_intrest = float(msg.data[0])
        print(f"[ DEBUG LOG ] Received partner's interest {partner_intrest}")
        reply_msg = TaskConvMsg() 
        if (partner_intrest > self.INTREST_THRESHOLD):
            print(f"[ DEBUG LOG ] Sending exec proposition to {msg.sender}")
            reply_msg.performative = MrsConvPerform.propose_exec_role
            reply_msg.data = [msg.sender]
        else:
            reply_msg.performative = MrsConvPerform.declare_coord_intrest
            temp_coord_intrest = str(self._task_fsm.interest_desc.coordination) #TODO: remove coord intrest at all, 
                                            # every agent should take part in supervising
            reply_msg.data = [temp_coord_intrest]
        if (self._task_fsm.interest_desc.execution <= self.INTREST_THRESHOLD):
            self._task_fsm.transition_to(SuperviseTask())
        reply_msg.short_id = msg.short_id
        return reply_msg
    
    def respond_to_exec_proposal(self, msg: TaskConvMsg):
        print(f"[ DEBUG LOG ] Received exec proposition from {msg.sender}")
        reply_msg = TaskConvMsg()
        reply_msg.short_id = msg.short_id
        reply_msg.performative = MrsConvPerform.accept_exec_proposal
        reply_msg.data = [msg.sender]
        self._task_fsm.transition_to(WaitForExec())
        return reply_msg
    

class WaitForExec(State):
    def change_state_routine(self):
        print("[ DEBUG LOG ] Moving directly to ExecTask")
        if (self._task_fsm._dependency_manager.are_dependencies_met()):
            self._task_fsm.transition_to(ExecTask())
        #else: wait for dependencies to be resolved

    def continue_after_resolved_dependencies(self):
        print("[ DEBUG LOG ] $$$$$$$ dependencies resolved $$$$$$ to ExecTask")
        self._task_fsm.transition_to(ExecTask())

class ExecTask(State):
    def change_state_routine(self):
        print("[ DEBUG LOG ] Executing task")
        # print("[ DEBUG LOG ] Moving to TaskCompleted")
        self._task_fsm.start_execution()
        # self._task_fsm.transition_to(TaskCompleted())

    def on_task_finished(self):
        print("[ DEBUG LOG ] Moving to TaskCompleted")
        self._task_fsm.transition_to(TaskCompleted())

    def respond_to_exec_info_request(self, msg: TaskConvMsg):
        return self._task_fsm._executor.get_task_execution_info()

class SuperviseTask(State):
    def change_state_routine(self):
        print("[ DEBUG LOG ] Supervising task")
        print("[ DEBUG LOG ] Moving to TaskCompleted")
        self._task_fsm.transition_to(TaskCompleted())

class TaskCompleted(State):
    def __init__(self) -> None:
        print('[ DEBUG LOG ] Task completed')

    def change_state_routine(self):
        self._task_fsm.inform_about_finished_dependency()