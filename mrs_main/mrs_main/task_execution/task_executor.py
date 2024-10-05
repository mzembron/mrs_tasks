from mrs_main.task_execution.concrete_executors.dummy_executor import DummyExecutor


class TaskExecutor:
    def __init__(self, task_desc: dict, callback_on_finish,  concrete_executor = None) -> None:
        """ Takes care of supervising the task execution by some
        external module """
        self._task_desc = task_desc

        # TODO: change concrete_executor to required argument, 
        # and use the concrete_executor object
        self._concrete_executor = DummyExecutor(callback_on_finish)
        

    def start_supervising_execution(self):
        """ entrypoint to trigger the task execution """
        self._concrete_executor.start_execution()

    def get_task_execution_info(self):
        """ returns the info regarding execution of the task,
        e.g. the estimated time to finish the task """
        self._concrete_executor.get_execution_info()