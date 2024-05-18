import rclpy
import common.constants as mrs_const

from rclpy.node import Node, Subscription, Publisher
from mrs_msgs.msg import TaskDesc, TaskConv
from tasks_management.task_manager import TaskManager
from tasks_management.task import Task
from common.objects import IntrestDescription, TopicSubPub
from common.conversation_data import MrsConvPerform


class OrdersManager(Node):
    """ OrdersManager - responsible for taking care of communication
    when it comes to task management.
    """
    def __init__(self, agent_name: str, task_manager: TaskManager):
        """
        """
        self.agent_name = agent_name
        self.node_name = 'orders_manager_'+agent_name
        super().__init__(node_name=self.node_name)

        self.subscription_task_def_topic = self.create_subscription(
            msg_type=TaskDesc,
            topic=mrs_const.TASKS_DEFINITION_TOPIC_NAME,
            callback=self.__task_definition_callback,
            qos_profile=10
        )

        self.task_topic_subpub_dict: dict[int, TopicSubPub] = {} 

        self.__task_manager = task_manager

    def __task_definition_callback(self, msg: TaskDesc):
        print(f'I heard task: {msg.type}')
        self.get_logger().info(f'I heard task: {msg.type}')
        task = Task(task_type=msg.type,
                    data=msg.data,
                    time_stamp=msg.stamp,
                    short_id=msg.short_id
                    )
        self.__create_sub_pub_for_task(task.short_id)
        intrest_estimation: IntrestDescription = self.__task_manager.append_task(task)
        self.__publish_intrest(task.short_id, intrest_estimation)

    def __create_sub_pub_for_task(self, task_id):

        dynamic_topic_sub_pub = TopicSubPub() 
        dynamic_topic_sub_pub.pub = self.create_publisher(TaskConv, '/mrs_main/id_' + str(task_id), 10)
        dynamic_topic_sub_pub.sub = self.create_subscription(
                                                msg_type=TaskConv,
                                                topic='/mrs_main/id_' + str(task_id),
                                                callback=self.__generic_task_callback,
                                                qos_profile=10
                                            )
        self.task_topic_subpub_dict[task_id] = dynamic_topic_sub_pub

    def __publish_intrest(self, task_id: int, intrest: IntrestDescription):
        task_conv_msg = TaskConv()
        task_conv_msg.performative = MrsConvPerform.declare_coord_intrest
        task_conv_msg.data = [str(intrest.coordination)]
        task_conv_msg.sender = self.agent_name
        pub: Publisher = self.task_topic_subpub_dict[task_id].pub
        pub.publish(task_conv_msg)

    def __generic_task_callback(self, msg: TaskConv):
        if msg.sender != self.agent_name:
            self.get_logger().info(f'I heard msg from {msg.sender}, performative: {msg.performative}, task data: {msg.data}')


if __name__ == '__main__':
    rclpy.init()

    ord_manager = OrdersManager('agent1')

    rclpy.spin(ord_manager)

    ord_manager.destroy_node()
    rclpy.shutdown()
