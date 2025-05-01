import threading
import time
from mrs_main.task_execution.concrete_executors.executor_interface import AbstractExecutor
from mrs_main.common.objects import TaskData
import subprocess
import random
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class TurtlebotExecutor(AbstractExecutor):
    """ class handles execution of tasks for specific type of agent,
    in this case it is a dummy executor, which does nothing """
    def __init__(self, callback_on_finish, orders_manager, agent_name, task_data: TaskData, task_exec_length=5) -> None:
        super().__init__(callback_on_finish)
        self.task_exec_length = task_exec_length
        self.orders_manager = orders_manager
        self.agent_name = agent_name
        self.execution_thread = threading.Thread(target=self.mock_task_execution)
        self.task_data = task_data

    def start_execution(self):
        """ entrypoint to trigger execution of task by external entity """
        
        
        print("[ DEBUG LOG ] !!!!!!!!!!!! Starting task execution !!!!!!!!!!!!")
        self.execution_thread.start()


    def get_execution_info(self):
        """ retrives info regarding task execution status """
        # retrieve info from separate thread mocking task execution
        pass

    def mock_task_execution(self):
        self.start_new_script('task_execution/concrete_executors/tb3_send_goal.py')
        # time.sleep(self.task_exec_length)
        self._on_execution_finished()

    def _on_execution_finished(self):
        """ callback method, called when task execution is finished """
        print("[ DEBUG LOG ] !!!!!!!!!!!! Task execution finished !!!!!!!!!!!!")
        self.callback_on_finish()

    def start_new_script(self, script_path):
        try:
            room_number = 1
            if (self.task_data.short_id < 4):
                return
            else:
                room_number = self.task_data.short_id - 3

            print("############ starting navigation ############")
            time.sleep(random.randint(1, 3))
            result = subprocess.run(['python3', script_path, self.agent_name, str(room_number)], capture_output=True, text=True)
            if result.returncode != 0:
                print("Script output:", result.stdout)
                logging.error(f"Script {script_path} failed with return code {result.returncode}")
                logging.error(f"Error output: {result.stderr}")
            else:
                print("Script error:", result.stderr)
                logging.info(f"Script {script_path} executed successfully")
                logging.info(f"Output: {result.stdout}")
        except Exception as e:
            print(f"An error occurred: {e}")
            logging.exception(f"An error occurred while running the script {script_path}")

    def __del__(self):
        """ Destructor to join the execution thread """
        if self.execution_thread is not None:
            self.execution_thread.join()