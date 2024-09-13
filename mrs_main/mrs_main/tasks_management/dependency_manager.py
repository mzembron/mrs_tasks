import networkx as nx

class DependencyManager:
    """ The main purpose of DependencyManager is to determine whether
        the given task can be executed immediately or if some other 
        dependencies should be resolved beforehand.  """
    def __init__(self, tasks_dict) -> None:
        self._tasks_dict = tasks_dict
        # self._tasks_dependencies: dict[int, list[int]] = {}
        self._tasks_dependencies = nx.DiGraph()

    def are_task_dependencies_met(self, task_id: int) -> bool:
        """ Checks if given task can be executed immediately """
        # A task can be executed if it has no incoming edges (no dependencies)
        return self._tasks_dependencies.in_degree(task_id) == 0
    
    def update_dependencies(self, finished_task_id: int):
        """ Updates the dependencies after receiving message regarding the task """
        # Remove the finished task from the graph
        if finished_task_id in self._tasks_dependencies:
            self._tasks_dependencies.remove_node(finished_task_id)

    def notify_task(self):
        """ sends signal to the task_fsm class to move on with handling the task,
            when all dependencies are resolved """
        pass

    #TODO: should include parameters defining dependencies 
    def introduce_task_dependencies(self, task_id: int, dependencies: list[int]):
        """ Define dependencies for new task """
        # self._tasks_dependencies[task_id] = dependencies
        # Add the task and its dependencies to the graph
        self._tasks_dependencies.add_node(task_id)
        for dep in dependencies:
            self._tasks_dependencies.add_edge(dep, task_id)

class TaskDependencyManager:
    def __init__(self, dependency_manager: DependencyManager, task_id: int, dependencies: list[int]) -> None:
        self.task_id = task_id
        self.__dependency_manager = dependency_manager
        self.__dependency_manager.introduce_task_dependencies(self.task_id, dependencies)

    
    def are_dependencies_met(self):
        return self.__dependency_manager.are_task_dependencies_met(self.task_id)
    
    def notify_on_finish(self):
        self.__dependency_manager.update_dependencies(finished_task_id=self.task_id)
