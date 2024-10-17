import json

from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import TaskConvMsg
# from tasks_management.task_manager import TaskManager #TODO: resolve circular import
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.dependency_manager import TaskDependencyManager
import mrs_main.common.constants as mrs_const

class TaskManagerInterface:
    def __init__(self, concrete_task_manager) -> None:
        """ Interface for interaction with the concrete TaskManager class,
            providing the base task handling functionalities: task state representation,
            definition of the next behavior (e.g. reply messages), etc. """
        from mrs_main.tasks_management.task_manager import TaskManager
        self.__concrete_task_manager: TaskManager = concrete_task_manager
        self._task_dict = self.__concrete_task_manager._task_dict
    
    @property
    def task_dict(self):
        return self._task_dict

    def receive_task(self, short_id: int, task_desc: str, task_finished_callback):
        """ Method receives the task info, creates the task object, and begins its management """
        task_desc_decoded = json.loads(task_desc)
        task_fsm = TaskFSM(dependency_manager=TaskDependencyManager(
                                dependency_manager=self.__concrete_task_manager._dependency_manager,
                                task_id=short_id,
                                dependencies = task_desc_decoded[mrs_const.TASK_DESC_DEPENDENCIES]),
                            task_desc=task_desc,
                            interest_desc=self.get_intrest(short_id),
                            task_finished_callback=task_finished_callback)
        print(f'[ DEBUG LOG ] Task of type: {task_desc}, received by TaskManager!')
        self._task_dict[short_id] = task_fsm

    def get_intrest(self, task_id: int):
        """ Returns the 'interest description' for the given task """
        # TODO: implement intrest calculation for every task
        return self.__concrete_task_manager.intrest_desc

    def define_next_behavior(self, task_conv_msg: TaskConvMsg) -> TaskConvMsg:
        """ Method defines next behavior for the given input message, which influences
            current state of the given task, return might be a reply message 
            or no response (None) """
        print(f'[ DEBUG LOG ] Received msg about task: {task_conv_msg.short_id}!')
        return self._task_dict[task_conv_msg.short_id].get_next_message(msg=task_conv_msg)
