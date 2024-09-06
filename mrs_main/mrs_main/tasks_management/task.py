from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.dependency_manager import TaskDependencyManager

class Task():
    def __init__(self, short_id: int, task_desc: list[str], dependency_manager: TaskDependencyManager) -> None:
        self.short_id: int = short_id
        self.desc: list[str] = task_desc
        self.fsm = TaskFSM(dependency_manager=dependency_manager)

    def get_response(self, msg: TaskConvMsg) -> TaskConvMsg:
        return self.fsm.get_next_message(msg=msg)

    @property
    def intrest(self) -> IntrestDescription:
        return self.fsm.intrest

    @intrest.setter
    def intrest(self, intrest: IntrestDescription) -> None:
        self.fsm.intrest = intrest


