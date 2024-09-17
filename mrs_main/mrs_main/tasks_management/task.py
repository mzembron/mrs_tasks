import json

from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.dependency_manager import TaskDependencyManager
from mrs_main.task_execution.task_executor import TaskExecutor

class Task():
    def __init__(self, short_id: int, task_desc: dict, dependency_manager: TaskDependencyManager) -> None:
        self.short_id: int = short_id
        self.desc: dict = task_desc
        self._executor = TaskExecutor(task_desc)
        self.fsm = TaskFSM(dependency_manager=dependency_manager, task_executor=self._executor)

    def get_response(self, msg: TaskConvMsg) -> TaskConvMsg:
        return self.fsm.get_next_message(msg=msg)

    @property
    def intrest(self) -> IntrestDescription:
        return self.fsm.intrest

    @intrest.setter
    def intrest(self, intrest: IntrestDescription) -> None:
        self.fsm.intrest = intrest

    def resume_after_finished_dependencies(self) -> None:
        self.fsm.inform_about_finished_dependency()


