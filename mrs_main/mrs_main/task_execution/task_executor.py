from mrs_main.tasks_management.task import Task

class TaskExecutor:
    def __init__(self) -> None:
        """ Takes care of supervising the task execution by some
        external module """
        pass

    def start_supervising_execution(self, task: Task):
        """ entrypoint to trigger the task execution """
        pass