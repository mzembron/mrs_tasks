import networkx as nx


class DependencyManager:
    
    """ The main purpose of DependencyManager is to determine whether
        the given task can be executed immediately or if some other 
        dependencies should be resolved beforehand """
    def __init__(self, tasks_dict) -> None:
        from mrs_main.tasks_management.task import Task
        self._tasks_dict: dict[int, Task] = tasks_dict
        self._tasks_dependencies = nx.DiGraph()

    def are_task_dependencies_met(self, task_id: int) -> bool:
        """ Checks if given task can be executed immediately """
        # A task can be executed if it has no incoming edges (no dependencies)
        return self._tasks_dependencies.in_degree(task_id) == 0
    
    def update_dependencies(self, finished_task_id: int):
        """ Updates the dependencies after receiving message regarding the task """
        if finished_task_id in self._tasks_dependencies:
            self._tasks_dependencies.nodes[finished_task_id]['finished'] = True
            # Extract all tasks that depend on the finished task
            dependent_tasks = [task for task in self._tasks_dependencies.successors(finished_task_id)]
            # Remove the edges between the finished task and its dependent tasks
            self._tasks_dependencies.remove_edges_from([(finished_task_id, task) for task in dependent_tasks])
            for task in dependent_tasks:
                # Check if the dependent task can be executed now
                if self.are_task_dependencies_met(task):
                    self.__notify_task_on_finish(task)
 
    def introduce_task_dependencies(self, task_id: int, dependencies: list[int]):
        """ Define dependencies for new task """
        # self._tasks_dependencies[task_id] = dependencies
        # Add the task and its dependencies to the graph
        self._tasks_dependencies.add_node(task_id, finished=False)
        for dep in dependencies:
            if not self._tasks_dependencies.nodes[dep]['finished']:
                self._tasks_dependencies.add_edge(dep, task_id)

    def __notify_task_on_finish(self, task_id: int):
        """ sends signal to the task_fsm class to move on with handling the task,
            when all dependencies are resolved """
        self._tasks_dict[task_id].resume_after_finished_dependencies()

class TaskDependencyManager:
    """ This class is the interface to interact with the DependencyManager in the scope of 
        a single, specific task """
    def __init__(self, dependency_manager: DependencyManager, task_id: int, dependencies: list[int]) -> None:
        self.task_id = task_id
        self.__dependency_manager = dependency_manager
        self.__dependency_manager.introduce_task_dependencies(self.task_id, dependencies)

    def are_dependencies_met(self):
        """ Check if managed task can be executed at the moment """
        return self.__dependency_manager.are_task_dependencies_met(self.task_id)
    
    def notify_on_finish(self):
        """ Propagate the info about finished task to the DependencyManager """
        self.__dependency_manager.update_dependencies(finished_task_id=self.task_id)
