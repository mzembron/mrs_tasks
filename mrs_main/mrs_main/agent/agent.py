import rclpy
import uuid
from mrs_main.communication.orders_manager import OrdersManager
from mrs_main.tasks_management.task_manager import TaskManager

class Agent():

    def __init__(self):
        agent_number = str(uuid.uuid1().int)
        self.agent_name = 'agent_' + agent_number
        self.__task_manager = TaskManager(agent_name=self.agent_name)
        self.__orders_manager = OrdersManager(agent_name=self.agent_name, task_manager=self.__task_manager)

    def start_agent(self):

        rclpy.spin(self.__orders_manager)

        self.__orders_manager.destroy_node()
        rclpy.shutdown()