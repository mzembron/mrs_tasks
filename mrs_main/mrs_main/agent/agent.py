import rclpy
from mrs_main.communication.orders_manager import OrdersManager

class Agent():

    def __init__(self, agent_number: str):
        self.agent_name = 'agent_' + agent_number
        self.__orders_manager = OrdersManager(self.agent_name)

    def start_agent(self):

        rclpy.spin(self.__orders_manager)

        self.__orders_manager.destroy_node()
        rclpy.shutdown()