

class DependencyManager:
    """ The main purpose of DependencyManager is to determine whether
        the given task can be executed immediately or if some other 
        dependencies should be resolved beforehand.  """
    def __init__(self) -> None:
        pass

    def verify_dependencies(self, task_id: int) -> bool:
        """ Checks if given task can be executed immediately """
        pass