from mrs_main.tasks_management.dependency_manager import DependencyManager

class Scheduler:
    def __init__(self, dependency_manager: DependencyManager):
        """ Manages the order and timing of task execution """
        self._dependency_manager = dependency_manager

    def get_next_task(self):
        """ Returns the next task to be executed """
        pass