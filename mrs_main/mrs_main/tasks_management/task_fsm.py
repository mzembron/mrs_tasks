from abc import ABC, abstractmethod
from typing import Callable, Type, Any

from mrs_main.common.objects import IntrestDescription, TaskConvMsg, TaskData
from mrs_main.common.conversation_data import MrsConvPerform
from mrs_main.common.exceptions import InvalidMsgPerformative
from mrs_main.task_execution.task_executor import TaskExecutor
from mrs_main.task_execution.concrete_executors.dummy_executor import DummyExecutor
from mrs_main.task_execution.concrete_executors.executor_interface import AbstractExecutor

import logging

logger = logging.getLogger(__name__)

class TaskFSM:

    def __init__(self, task_data: TaskData,
                    interest_desc: IntrestDescription,
                    task_finished_callback: Callable[..., Any],
                    agent_selected_callaback: Callable[..., Any],
                    state_changed_callback: Callable[..., Any],
                    async_task_msg_callback: Callable[..., Any],
                    orders_manager,
                    knowledge_base,
                    concrete_executor: Type[AbstractExecutor]=DummyExecutor,
                    agent_name: str = ''
                    ) -> None:
        self._state = None
        self.state_change_callback = state_changed_callback
        self.async_task_msg_callback = async_task_msg_callback
        self.transition_to(DefineEstimate())
        self._state.state_data['estimations'] = {}
        self._state.state_data['estimations'][agent_name] = interest_desc.execution
        self._state.state_data['votes'] = {}
        self._executor = TaskExecutor(task_data, self.receive_task_finished_signal, concrete_executor, orders_manager, agent_name=agent_name)
        self.orders_manager = orders_manager
        self.task_data = task_data
        self.interest_desc = interest_desc
        self.task_finished_callback = task_finished_callback
        self.agent_selected_callaback = agent_selected_callaback
        self.agent_name = agent_name
        self.knowledge_base = knowledge_base

    @property
    def current_state(self) -> str:
        return self._state.__class__.__name__

    def get_next_message(self, msg: TaskConvMsg):
        """ Get response (or no response) to the received message based on the current state """
        return self._state.define_next(msg)

    def transition_to(self, state):
        """ Change the state of the task FSM """
        state_data = {}
        if self._state is not None: state_data = self._state.state_data
        self._state = state
        self._state.task_fsm = self
        self._state.state_data = state_data
        self.state_change_callback(state.__class__.__name__, self._state.state_data)
        self._state.change_state_routine()
         # clear the state data
    
    def update_state_from_alignment(self, new_state, state_data):
        self._state.state_data = state_data
        if new_state == 'SuperviseTask':
            self.transition_to(SuperviseTask())
        elif new_state == 'TaskCompleted':
            self.transition_to(TaskCompleted())
    
    def resume_after_finished_dependencies(self) -> None:
        """ Resume task (move to task-execution state) after all dependencies are resolved """
        self._state.continue_after_resolved_dependencies()
    
    def start_execution(self) -> None:
        """ Trigger execution of the specific task by the executor module"""
        self._executor.start_supervising_execution()

    def send_async_msg(self, msg: TaskConvMsg) -> None:
        self.async_task_msg_callback(msg)

    def handle_task_finished(self):
        """ Perform actions after the task is finished:
            - notify the dependency manager
            - call the callback function to inform the task manager that the task is finished """
        self.task_finished_callback(self.task_data)

    def receive_task_finished_signal(self):
        """ Callback to trigger the transition to the TaskCompleted state after the task is finished """
        self._state.on_task_finished()

    def handle_estimations_mismatch(self, incoming_estimations):
        """ Handle the case when the estimations of the task execution time are mismatched """
        self._state.update_estimations_after_mismatch(incoming_estimations)

class State(ABC):
    state_data = {}
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
        elif (msg.performative == MrsConvPerform.inform_task_finished):
            return self.respond_to_task_finished_info(msg)
        else:
            raise InvalidMsgPerformative

    def change_state_routine(self):
        """ transition method, allows for state specific behavior on transition """    
        pass

# virtual methods
    def continue_after_resolved_dependencies(self):
        """ Method to be called once the dependencies are resolved """
        return
    
    def on_task_finished(self):
        """ Callback method to be called once thetask execution has finished """
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

    def respond_to_task_finished_info(self, msg: TaskConvMsg):
        return
    
# virtual methods - respond to knowledge mismatches
    def update_estimations_after_mismatch(self, incoming_estimations):
        return

class DefineEstimate(State):
    def respond_to_coord_intrest_declaration(self, msg: TaskConvMsg) -> TaskConvMsg:
        partner_intrest = float(msg.data[0])
        # # TEMPORARY: for alignment algorithm development
        # if msg.short_id > 4:
        #     return
        print(f"[ DEBUG LOG ] Received partner's interest {partner_intrest}")
        self.state_data['estimations'][msg.sender]  = partner_intrest
        reply_msg = TaskConvMsg() 
        # self.state_data
        # if (msg.short_id<7) or (( len([key for key in self.state_data['estimations']]))>2):
        if ( len([key for key in self.state_data['estimations']]))>self._task_fsm.knowledge_base.get_current_agent_number():
            best_executor = min(self.state_data['estimations'], key=self.state_data['estimations'].get)
            print(f"[ DEBUG LOG ] Sending exec proposition of task {msg.short_id} to {best_executor}")
            reply_msg.performative = MrsConvPerform.propose_exec_role
            reply_msg.data = [best_executor]
            reply_msg.sender = self._task_fsm.agent_name
            return reply_msg
        # if (partner_intrest > self.INTREST_THRESHOLD):
        #     print(f"[ DEBUG LOG ] Sending exec proposition of task {msg.short_id} to {msg.sender}")
        #     reply_msg.performative = MrsConvPerform.propose_exec_role
        #     reply_msg.data = [msg.sender]
        #     return reply_msg
        else:
            return
    
    def respond_to_exec_proposal(self, msg: TaskConvMsg):
        logger.info(f"[ DEBUG LOG ] Received exec proposition from {msg.sender}")
        if (str(msg.data[0]) == self._task_fsm.agent_name):
            logger.info('[ DEBUG LOG ] %%%%%%%%%%%%%%%% Accepting Task %%%%%%%%%%%%%%%%')
            reply_msg = TaskConvMsg()
            reply_msg.short_id = msg.short_id
            reply_msg.performative = MrsConvPerform.accept_exec_proposal
            reply_msg.data = [msg.sender]
            self.state_data['executor'] = self._task_fsm.agent_name
            self._task_fsm.transition_to(WaitForExec())
            return reply_msg
        else:
            return None
        
    def respond_to_exec_acceptance(self, msg):
        if(msg.sender != self._task_fsm.agent_name):
            # if (self._task_fsm.interest_desc.execution <= self.INTREST_THRESHOLD):
            self.state_data['executor'] = msg.sender
            self._task_fsm.transition_to(SuperviseTask())
    
    def update_estimations_after_mismatch(self, incoming_estimations):
        logger.warning(f'Updating estimations for task {self._task_fsm.task_data.short_id}')
        for key, value in  incoming_estimations.items():
            logger.info(f"Estimation for {key}: {value}")

        for robot_name, estimations in incoming_estimations.items():
            if robot_name not in self.state_data['estimations']:
                self.state_data['estimations'][robot_name] = estimations

        if (len([key for key in self.state_data['estimations']]))>self._task_fsm.knowledge_base.get_current_agent_number():
            reply_msg = TaskConvMsg()
            best_executor = min(self.state_data['estimations'], key=self.state_data['estimations'].get)
            # logger.info(f"@@@ Would send exec proposition of task {self.task_fsm.task_data.short_id} to {best_executor}")
            reply_msg.performative = MrsConvPerform.propose_exec_role
            reply_msg.data = [best_executor]
            reply_msg.short_id = self.task_fsm.task_data.short_id
            reply_msg.sender = self._task_fsm.agent_name
            #TODO: change to callback in TaskManager 
            # self.task_fsm.orders_manager.__generic_async_task_msg_callback(reply_msg)
            self.task_fsm.send_async_msg(reply_msg)
            return
        
    

class WaitForExec(State):
    def change_state_routine(self):
        print("[ DEBUG LOG ] Moving directly to ExecTask")
        self._task_fsm.agent_selected_callaback()

    def continue_after_resolved_dependencies(self):
        print("[ DEBUG LOG ] $$$$$$$ dependencies resolved $$$$$$ to ExecTask")
        self._task_fsm.transition_to(ExecTask())
    
    def respond_to_exec_proposal(self, msg: TaskConvMsg):
        pass
        # print(f"[ DEBUG LOG ] Already assigned to task {msg.short_id} . ignoring!")

class ExecTask(State):
    def change_state_routine(self):
        print("[ DEBUG LOG ] Executing task")
        self._task_fsm.start_execution()

    def on_task_finished(self):
        print("[ DEBUG LOG ] Moving to TaskCompleted")
        self._task_fsm.transition_to(TaskCompleted())

    def respond_to_exec_info_request(self, msg: TaskConvMsg):
        return self._task_fsm._executor.get_task_execution_info()
    
    def respond_to_exec_proposal(self, msg: TaskConvMsg):
        print(f"[ DEBUG LOG ] Already assigned to task {msg.short_id} . ignoring!")

class SuperviseTask(State):
    def change_state_routine(self):
        print("[ DEBUG LOG ] Supervising task")
    
    def respond_to_task_finished_info(self, msg: TaskConvMsg):
        print("[ DEBUG LOG ] Moving to TaskCompleted")
        self._task_fsm.transition_to(TaskCompleted())
    
    def respond_to_exec_proposal(self, msg: TaskConvMsg):
        print(f"[ DEBUG LOG ] Cannot execute this task! Other agent is executing task {msg.short_id} ! ignoring!")

class TaskCompleted(State):
    def __init__(self) -> None:
        print('[ DEBUG LOG ] Task completed')

    def change_state_routine(self):
        self._task_fsm.handle_task_finished()