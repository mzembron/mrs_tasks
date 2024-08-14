import rclpy
import common.constants as mrs_const

from rclpy.node import Node, Subscription, Publisher
from mrs_msgs.msg import TaskDesc, TaskConv
from tasks_management.task_manager_interface import TaskManagerInterface
from tasks_management.task import Task
from common.objects import IntrestDescription, TopicSubPub, TaskConvMsg
from common.conversation_data import MrsConvPerform


class OrdersManager(Node):
    """ Orders Manager takes care of communication in the contexts of 
    diffrent tasks - every task has its own ROS topic.
    """
    def __init__(self, agent_name: str, task_manager: TaskManagerInterface):
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
        self.get_logger().info(f'I heard task: {msg.type}')
        task = Task(short_id=msg.short_id, task_desc=msg.data)
        self.__create_sub_pub_for_task(task.short_id)
        self.__task_manager.receive_task(task)
        intrest_estimation: IntrestDescription = self.__task_manager.get_intrest(task.short_id)
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
        task_conv_msg.short_id = task_id
        task_conv_msg.sender = self.agent_name
        pub: Publisher = self.task_topic_subpub_dict[task_id].pub
        pub.publish(task_conv_msg)

    def __generic_task_callback(self, msg: TaskConv):
        if msg.sender != self.agent_name:
            self.get_logger().info(f'I heard msg from {msg.sender}, \
                                   performative: {msg.performative}, task data: {msg.data}')
            conv_msg = TaskConvMsg()
            conv_msg.deserialize(msg=msg)
            answer_msg = self.__task_manager.define_next_behavior(conv_msg)
            if (answer_msg is None): return
            answer_msg.add_conversation_context(sender_name=self.agent_name, id=conv_msg.short_id)
            print(f'Answer msg data {answer_msg.data[0]}')
            conv_answer_msg= answer_msg.serialize()
            self.task_topic_subpub_dict[msg.short_id].pub.publish(conv_answer_msg)


if __name__ == '__main__':
    rclpy.init()

    ord_manager = OrdersManager('agent1')

    rclpy.spin(ord_manager)

    ord_manager.destroy_node()
    rclpy.shutdown()
