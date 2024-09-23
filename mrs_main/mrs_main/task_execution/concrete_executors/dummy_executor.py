from mrs_main.task_execution.concrete_executors.executor_interface import AbstractExecutor

class DummyExecutor(AbstractExecutor):
    """ class handles execution of tasks for specific type of agent,
    in this case it is a dummy executor, which does nothing """
    def __init__(self) -> None:
        pass

    def start_execution(self):
        pass

    def get_execution_info(self):
        pass