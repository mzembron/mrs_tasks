import json
from functools import partial

from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg,  TaskData
from mrs_main.tasks_management.dependency_manager import DependencyManager, TaskDependencyManager
from mrs_main.tasks_management.scheduler import Scheduler

class TaskManager:
    def __init__(self, agent_name: str, intrest_exec: float = 0.2, intrest_coord: float = 0.2) -> None:
        """ Interface for interaction with the concrete TaskManager class,
            providing the base task handling functionalities: task state representation,
            definition of the next behavior (e.g. reply messages), etc. """
        self.agent_name: str = agent_name
        self.intrest_desc = IntrestDescription()
        self.intrest_desc.execution = intrest_exec
        self.intrest_desc.coordination = intrest_coord
        self._task_dict: dict[int, TaskFSM] = {} # all sensed tasks, not only the ones handled by this agent
        self._dependency_manager = DependencyManager(self._task_dict)
        self._scheduler = Scheduler(self._dependency_manager)
    
    @property
    def task_dict(self):
        return self._task_dict

    def receive_task(self, short_id: int, task_desc: str, task_finished_callback):
        """ Method receives the task info, creates the task object, and begins its management """
        task_data = TaskData.from_task_definition(short_id, task_desc)
        callback_with_task_id = partial(self.__agent_selected_to_execute_callback, short_id)
        task_finished_callback_extended = lambda task_data: (task_finished_callback(task_data),
                                                            self._scheduler.handle_current_task_finished(short_id))
                                            # task_data will be passed to lambda by the TaskFSM
        task_fsm = TaskFSM(dependency_manager=TaskDependencyManager(
                                dependency_manager=self._dependency_manager,
                                task_id=short_id,
                                dependencies = task_data.dependencies),
                            task_data=task_data,
                            interest_desc=self.get_intrest(short_id),
                            task_finished_callback=task_finished_callback_extended,
                            agent_selected_callaback=callback_with_task_id
                            )
        print(f'[ DEBUG LOG ] Task of type: {task_desc}, received by TaskManager!')
        self._task_dict[short_id] = task_fsm

    def get_intrest(self, task_id: int):
        """ Returns the 'interest description' for the given task """
        # TODO: implement intrest calculation for every task
        return self.intrest_desc

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

