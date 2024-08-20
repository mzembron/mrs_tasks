from mrs_main.tasks_management.task import Task
# from tasks_management.task_manager import TaskManager #TODO: resolve circular import
from mrs_main.common.objects import IntrestDescription, TaskConvMsg

class TaskManagerInterface:
    def __init__(self, task_dict, concrete_task_manager) -> None:
        self._task_dict = task_dict
        from mrs_main.tasks_management.task_manager import TaskManager
        self.__concrete_task_manager: TaskManager = concrete_task_manager
    
    @property
    def task_dict(self):
        return self._task_dict

    def receive_task(self, task: Task):
        print(f'[ DEBUG LOG ] Task of type: {task.desc}, received by TaskManager!')
        self._task_dict[task.short_id] = task

    def get_intrest(self, task_id: int):
        # TODO: implement intrest calculation for every task
        return self.__concrete_task_manager.intrest_desc

    def define_next_behavior(self, task_conv_msg: TaskConvMsg) -> TaskConvMsg:
        print(f'[ DEBUG LOG ] Received msg about task: {task_conv_msg.short_id}!')
        return self._task_dict[task_conv_msg.short_id].get_response(task_conv_msg)