from tasks_management.task import Task
from common.objects import IntrestDescription
import random

class TaskManager():
    def __init__(self, agent_name: str, intrest_exec: float = 0.2, intrest_coord: float = 0.2):
        self.agent_name: str = agent_name
        self.intrest_exec = intrest_exec
        self.intrest_coord = intrest_coord
        self.task_list: list[Task] = []
    
    def append_task(self, task: Task):
        print(f'Task of type: {task.task_type}, received by TaskManager!')
        self.task_list.append(task)
        intrest_description = IntrestDescription()
        intrest_description.execution = self.intrest_exec
        intrest_description.coordination = self.intrest_coord
        return intrest_description
    
    def define_next_behaviour(self, task_id):
        pass
