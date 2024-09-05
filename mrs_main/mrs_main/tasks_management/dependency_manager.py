

class DependencyManager:
    """ The main purpose of DependencyManager is to determine whether
        the given task can be executed immediately or if some other 
        dependencies should be resolved beforehand.  """
    def __init__(self, tasks_dict) -> None:
        self._tasks_dict = tasks_dict

    def verify_dependencies(self, task_id: int) -> bool:
        """ Checks if given task can be executed immediately """
        pass
    
    def update_dependencies(self, task_id: int):
        """ Updates the dependencies after receiving message regarding the task """
        pass

    def notify_task(self):
        """ sends signal to the task_fsm class to move on with handling the task,
            when all dependencies are resolved """
        pass

    #TODO: should include parameters defining dependencies 
    def introduce_task_dependencies(self):
        """ Define dependencies for new task """
        pass