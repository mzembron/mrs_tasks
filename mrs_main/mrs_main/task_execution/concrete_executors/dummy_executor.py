import threading
import time
from mrs_main.task_execution.concrete_executors.executor_interface import AbstractExecutor

class DummyExecutor(AbstractExecutor):
    """ class handles execution of tasks for specific type of agent,
    in this case it is a dummy executor, which does nothing """
    def __init__(self, callback_on_finish) -> None:
        super().__init__(callback_on_finish)
        self.execution_thread = None

    def start_execution(self):
        """ entrypoint to trigger execution of task by external entity """
        #TODO: start mock of task execution in separate thread

        
        self.execution_thread = threading.Thread(target=self.mock_task_execution)
        self.execution_thread.start()


    def get_execution_info(self):
        """ retrives info regarding task execution status """
        # retrieve info from separate thread mocking task execution
        pass

    def mock_task_execution(self):
        time.sleep(5)
        self._on_execution_finished()

    def _on_execution_finished(self):
        """ callback method, called when task execution is finished """
        print("[ DEBUG LOG ] Task execution finished.")
        self.callback_on_finish()


    def __del__(self):
        """ Destructor to join the execution thread """
        if self.execution_thread is not None:
            self.execution_thread.join()
            print("[ DEBUG LOG ] Dummy execution thread joined.")