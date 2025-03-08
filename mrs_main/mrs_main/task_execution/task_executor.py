from mrs_main.common.objects import TaskData
from mrs_main.task_execution.concrete_executors.turtlebot_executor import TurtlebotExecutor

class TaskExecutor:
    def __init__(self, task_data: TaskData, callback_on_finish, concrete_executor, orders_manager, agent_name) -> None:
        """ Takes care of supervising the task execution by some
        external module """
        self._task_data: TaskData = task_data
        # concrete_executor = TurtlebotExecutor(callback_on_finish)
        # TODO: change concrete_executor to required argument, 
        # and use the concrete_executor object
        # self._concrete_executor = concrete_executor(callback_on_finish)
        self._concrete_executor = TurtlebotExecutor(callback_on_finish, orders_manager=orders_manager, agent_name=agent_name, task_data=task_data)

        

    def start_supervising_execution(self):
        """ entrypoint to trigger the task execution """
        self._concrete_executor.start_execution()

    def get_task_execution_info(self):
        """ returns the info regarding execution of the task,
        e.g. the estimated time to finish the task """
        self._concrete_executor.get_execution_info()