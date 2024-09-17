
class TaskExecutor:
    def __init__(self, task_desc: dict) -> None:
        """ Takes care of supervising the task execution by some
        external module """
        self._task_desc = task_desc

    def start_supervising_execution(self):
        """ entrypoint to trigger the task execution """
        pass