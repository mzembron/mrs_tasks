import rclpy
import uuid
from mrs_main.communication.orders_manager import OrdersManager
from mrs_main.tasks_management.task_manager import TaskManager


class Agent():

    def __init__(self, intrest_exec: float = 0.2, intrest_coord: float = 0.2):
        agent_number = str(uuid.uuid1().int)
        self.agent_name = 'agent_' + agent_number
        self.__task_manager = TaskManager(agent_name=self.agent_name,
                                          intrest_exec=intrest_exec,
                                          intrest_coord=intrest_coord)
        self.__orders_manager = OrdersManager(agent_name=self.agent_name,
                                              task_manager=self.__task_manager.interface)

    def start_agent(self):

        rclpy.spin(self.__orders_manager)

        self.__orders_manager.destroy_node()
        rclpy.shutdown()

    def print_task_states(self):
        for task_id, task in self.__task_manager._task_dict.items():
            state_class = type(task.fsm._state).__name__
            print(f"Task ID: {task_id}, State: {state_class}")
