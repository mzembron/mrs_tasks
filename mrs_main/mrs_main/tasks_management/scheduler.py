from mrs_main.tasks_management.dependency_manager import DependencyManager
from mrs_main.tasks_management.task import Task

class Scheduler:
    def __init__(self, dependency_manager: DependencyManager):
        """ Manages the order and timing of task execution """
        self._dependency_manager = dependency_manager
        self.backlog = []

    def append_task(self, task: Task):
        """ Appends a new task to the task queue """
        self.backlog.append(task)

    def get_next_task(self):
        """ Returns the next task to be executed """
        pass