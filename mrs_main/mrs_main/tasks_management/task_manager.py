from mrs_main.tasks_management.task import Task


class TaskManager():
    def __init__(self, agent_name: str):
        self.agent_name: str = agent_name
        self.task_list: list[Task] = []
    
    def append_task(self, task: Task):
        print(f'Task of type: {task.task_type}, received by TaskManager!')
        self.task_list.append(task)
