import json
from functools import partial

from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg,  TaskData
from mrs_main.tasks_management.dependency_manager import DependencyManager
from mrs_main.tasks_management.scheduler import Scheduler
from mrs_main.knowledge_base.knowledge_base import KnowledgeBase

class TaskManager:
    def __init__(self, agent_name: str, agent_type: int = 0, start_scheduler_kicking_thread= True) -> None:
        """ Interface for interaction with the concrete TaskManager class,
            providing the base task handling functionalities: task state representation,
            definition of the next behavior (e.g. reply messages), etc. """
        self.agent_name: str = agent_name
        self._task_dict: dict[int, TaskFSM] = {} # all sensed tasks, not only the ones handled by this agent
        self._task_states_list = ['init']
        self._dependency_manager = DependencyManager(self._task_dict)
        self._scheduler = Scheduler(self._dependency_manager, start_kicking_thread=start_scheduler_kicking_thread)
        self._knowledge_base = KnowledgeBase(agent_type)
    
    @property
    def task_dict(self):
        return self._task_dict

    def receive_task(self, short_id: int, task_desc: str, task_finished_callback, orders_manager):
        """ Method receives the task info, creates the task object, and begins its management """
        task_data = TaskData.from_task_definition(short_id, task_desc)
        self._dependency_manager.introduce_task_dependencies(short_id, task_data.dependencies)
        callback_with_task_id = partial(self.__agent_selected_to_execute_callback, short_id)
        self._task_states_list.append('init')
        callback_state_changed = partial(self.change_task_state_in_dict, short_id)
        task_finished_callback_extended = lambda task_data: (task_finished_callback(task_data),
                                                            self._scheduler.handle_current_task_finished(short_id),
                                                            self._dependency_manager.update_dependencies(short_id))
                                            # task_data will be passed to lambda by the TaskFSM
        task_fsm = TaskFSM( task_data=task_data,
                            interest_desc=self._knowledge_base.get_intrest_desc(task_data), # input 
                            task_finished_callback=task_finished_callback_extended,
                            agent_selected_callaback=callback_with_task_id,
                            state_changed_callback=callback_state_changed,
                            orders_manager=orders_manager,
                            agent_name=self.agent_name
                            )
        print(f'[ DEBUG LOG ] Task of type: {task_desc}, received by TaskManager!')
        self._task_dict[short_id] = task_fsm

    def get_intrest(self, task_id: int):
        """ Returns the 'interest description' for the given task """
        # TODO: implement intrest calculation for every task
        return self._knowledge_base.get_intrest_desc(self._task_dict[task_id].task_data) # TODO: perform checks if the task is not exceeding the dict size

    def define_next_behavior(self, task_conv_msg: TaskConvMsg) -> TaskConvMsg:
        """ Method defines next behavior for the given input message, which influences
            current state of the given task, return might be a reply message 
            or no response (None) """
        print(f'[ DEBUG LOG ] Received msg about task: {task_conv_msg.short_id}!')
        return self._task_dict[task_conv_msg.short_id].get_next_message(msg=task_conv_msg)
    
    def __agent_selected_to_execute_callback(self, task_id: int):
        """ Method called when the agent is selected to execute the task """
        print(f'[ DEBUG LOG ] Task {task_id} appended to scheduler!')
        self._scheduler.append_task(self._task_dict[task_id]) # from now on scheduler manages the task FSM

    def change_task_state_in_dict(self, task_id: int, new_state):
        """ Changes the state of the task in the task dict """
        for x in self._task_states_list: print(x)

        self._task_states_list[task_id] = str(new_state)

    def get_states_list(self):
        return self._task_states_list
