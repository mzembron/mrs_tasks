import rclpy
from rclpy.node import Node
from mrs_msgs.msg import TaskDesc
import mrs_main.common.constants as mrs_const
from mrs_main.tasks_management.task_manager import TaskManager
from mrs_main.tasks_management.task import Task


class OrdersManager(Node):
    """ OrdersManager - responsible for taking care of communication
    when it comes to task management.
    """
    def __init__(self, agent_name: str, task_manager: TaskManager):
        """
        """
        self.node_name = 'orders_manager_'+agent_name
        super().__init__(node_name=self.node_name)

        self.subscription_task_def_topic = self.create_subscription(
            msg_type=TaskDesc,
            topic=mrs_const.TASKS_DEFINITION_TOPIC_NAME,
            callback=self.__task_definition_callback,
            qos_profile=10
        )

        self.__task_manager = task_manager

    def __task_definition_callback(self, msg: TaskDesc):
        print(f'I heard task: {msg.type}')
        self.get_logger().info(f'I heard task: {msg.type}')
        task = Task(task_type=msg.type,
                    data=msg.data,
                    time_stamp=msg.stamp,
                    priority=msg.priority
                    )

        self.__task_manager.append_task(task)


if __name__ == '__main__':
    rclpy.init()

    ord_manager = OrdersManager('agent1')

    rclpy.spin(ord_manager)

    ord_manager.destroy_node()
    rclpy.shutdown()
