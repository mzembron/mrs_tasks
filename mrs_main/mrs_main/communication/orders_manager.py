import rclpy
from rclpy.node import Node
from mrs_msgs.msg import TaskDesc
import mrs_main.common.constants as mrs_const


class OrdersManager(Node):
    def __init__(self, agent_name: str):
        self.node_name = 'orders_manager_'+agent_name
        super().__init__(node_name=self.node_name)

        self.subscription_task_def_topic = self.create_subscription(
            msg_type=TaskDesc,
            topic=mrs_const.TASKS_DEFINITION_TOPIC_NAME,
            callback=self.__task_definition_callback,
            qos_profile=10
        )

    def __task_definition_callback(self, msg: TaskDesc):
        print(f'I heard task: {msg.type}')
        self.get_logger().info(f'I heard task: {msg.type}')


if __name__ == '__main__':
    rclpy.init()

    ord_manager = OrdersManager('agent1')

    rclpy.spin(ord_manager)

    ord_manager.destroy_node()
    rclpy.shutdown()
