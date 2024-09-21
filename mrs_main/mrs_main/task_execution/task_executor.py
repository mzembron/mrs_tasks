
class TaskExecutor:
    def __init__(self, task_desc: dict) -> None:
        """ Takes care of supervising the task execution by some
        external module """
        self._task_desc = task_desc

    def start_supervising_execution(self):
        """ entrypoint to trigger the task execution """
        pass

    def get_task_execution_info(self):
        """ returns the info regarding execution of the task;
        e.g. the estimated time to finish the task """
        pass