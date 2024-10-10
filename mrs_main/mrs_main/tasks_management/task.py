import json

from typing import Callable, Any
from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.dependency_manager import TaskDependencyManager

class Task():
    def __init__(self, short_id: int, task_desc: dict,
                    dependency_manager: TaskDependencyManager,
                    interest_desc: IntrestDescription,
                    task_finished_callback: Callable[..., Any]) -> None:
        self.short_id: int = short_id
        self.desc: dict = task_desc
        self.fsm = TaskFSM(dependency_manager=dependency_manager, task_desc=self.desc, interest_desc=interest_desc, task_finished_callback=task_finished_callback)

    def get_response(self, msg: TaskConvMsg) -> TaskConvMsg:
        return self.fsm.get_next_message(msg=msg)

    @property
    def intrest(self) -> IntrestDescription:
        return self.fsm.interest_desc

    @intrest.setter
    def intrest(self, intrest: IntrestDescription) -> None:
        self.fsm.interest_desc = intrest

    def resume_after_finished_dependencies(self) -> None:
        """ Resume task after all dependencies are resolved """
        self.fsm.resume_after_finished_dependencies()


