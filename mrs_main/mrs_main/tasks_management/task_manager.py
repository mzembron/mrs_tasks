from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.tasks_management.task_manager_interface import TaskManagerInterface
from mrs_main.tasks_management.dependency_manager import DependencyManager
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
import random

class TaskManager():

    def __init__(self, agent_name: str, intrest_exec: float = 0.2, intrest_coord: float = 0.2):
        self.agent_name: str = agent_name
        self.intrest_desc = IntrestDescription()
        self.intrest_desc.execution = intrest_exec
        self.intrest_desc.coordination = intrest_coord
        self._task_dict: dict[int, TaskFSM] = {} # all sensed tasks, not only the ones handled by this agent
        self._interface = TaskManagerInterface(concrete_task_manager=self)
        self._dependency_manager = DependencyManager(self._task_dict)

    @property
    def interface(self):
        return self._interface
