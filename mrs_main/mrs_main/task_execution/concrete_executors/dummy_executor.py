import threading
import time
from mrs_main.task_execution.concrete_executors.executor_interface import AbstractExecutor

class DummyExecutor(AbstractExecutor):
    """ class handles execution of tasks for specific type of agent,
    in this case it is a dummy executor, which does nothing """
    def __init__(self) -> None:
        self.execution_thread = None

    def start_execution(self):
        """ entrypoint to trigger execution of task by external entity """
        #TODO: start mock of task execution in separate thread
        def mock_task_execution():
            time.sleep(5)
            print("[ DEBUG LOG ] Task execution completed.")
            self._on_execution_finished()
        
        self.execution_thread = threading.Thread(target=mock_task_execution)
        self.execution_thread.start()


    def get_execution_info(self):
        """ retrives info regarding task execution status """
        # retrieve info from separate thread mocking task execution
        pass

    def _on_execution_finished(self):
        """ callback method, called when task execution is finished """
        print("[ DEBUG LOG ] Task execution finished.")