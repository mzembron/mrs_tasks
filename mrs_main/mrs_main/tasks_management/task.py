from tasks_management.task_fsm import TaskFSM
from common.objects import IntrestDescription, TaskConvMsg

class Task():
    def __init__(self, short_id: int, task_desc: list[str]):
        self.short_id: int = short_id
        self.desc: list[str] = task_desc
        self.fsm = TaskFSM()

    def get_response(self, msg: TaskConvMsg) -> TaskConvMsg:
        return self.fsm.get_next_message(msg=msg)

    @property
    def intrest(self) -> IntrestDescription:
        return self.fsm.intrest

    @intrest.setter
    def intrest(self, intrest: IntrestDescription) -> None:
        self.fsm.intrest = intrest


