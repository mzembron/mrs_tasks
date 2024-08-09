import rclpy
import uuid
from communication.orders_manager import OrdersManager
from tasks_management.task_manager import TaskManager


class Agent():

    def __init__(self, intrest_exec: float = 0.2, intrest_coord: float = 0.2):
        agent_number = str(uuid.uuid1().int)
        self.agent_name = 'agent_' + agent_number
        self.__task_manager = TaskManager(agent_name=self.agent_name,
                                          intrest_exec=0.6,
                                          intrest_coord=0.4)
        self.__orders_manager = OrdersManager(agent_name=self.agent_name,
                                              task_manager=self.__task_manager.interface)

    def start_agent(self):

        rclpy.spin(self.__orders_manager)

        self.__orders_manager.destroy_node()
        rclpy.shutdown()
