from tasks_management.task import Task
from common.objects import IntrestDescription, TaskConvMsg
import random

class TaskManager():

    def __init__(self, agent_name: str, intrest_exec: float = 0.2, intrest_coord: float = 0.2):
        self.agent_name: str = agent_name
        self.intrest_desc = IntrestDescription()
        self.intrest_desc.execution = intrest_exec
        self.intrest_desc.coordination = intrest_coord
        self.id_task_dict: dict[int, Task] = {}
    
    def append_task(self, task: Task):
        print(f'Task of type: {task.desc}, received by TaskManager!')
        self.id_task_dict[task.short_id] = task
        return self.intrest_desc
    
    def define_next_behaviour(self, task_conv_msg: TaskConvMsg) -> TaskConvMsg:
        print(f'Recived msg about task: {task_conv_msg.short_id}!')
        return self.id_task_dict[task_conv_msg.short_id].get_response(task_conv_msg)
        
