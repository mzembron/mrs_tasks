import rclpy
import mrs_main.common.constants as mrs_const

from rclpy.node import Node, Publisher
from mrs_msgs.msg import TaskDesc, TaskConv
from mrs_main.tasks_management.task_manager_interface import TaskManagerInterface
from mrs_main.common.objects import IntrestDescription, TopicSubPub, TaskConvMsg
from mrs_main.common.conversation_data import MrsConvPerform

from rclpy.qos import QoSProfile, ReliabilityPolicy

class OrdersManager(Node):
    """ Orders Manager takes care of communication in the contexts of 
    diffrent tasks - every task has its own ROS topic.
    """
    def __init__(self, agent_name: str, task_manager: TaskManagerInterface):
        """
        Attributes:
            agent_name (str)
            node_name (str): The name of the node, derived from the agent name.
            subscription_task_def_topic (Subscription): ROS2 subscription to the task definition topic.
            task_topic_subpub_dict (dict[int, TopicSubPub]): Dictionary mapping task IDs to
                    their respective TopicSubPub objects (ROS2 subscription and publisher).
            __task_manager (TaskManagerInterface): The task manager instance, allows accessing 
                    and managing the state of tasks.
        """
        self.agent_name = agent_name
        self.node_name = 'orders_manager_'+agent_name
        super().__init__(node_name=self.node_name)
        self._qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription_task_def_topic = self.create_subscription(
            msg_type=TaskDesc,
            topic=mrs_const.TASKS_DEFINITION_TOPIC_NAME,
            callback=self.__task_definition_callback,
            qos_profile=self._qos_profile
        )

        self.task_topic_subpub_dict: dict[int, TopicSubPub] = {} 

        self.__task_manager = task_manager

    def __task_definition_callback(self, msg: TaskDesc):
        """ Callback for the generic topicwith defintion of any task (action entrypoint)"""
        self.get_logger().info(f'I heard task: {msg.type}')
        
        self.__create_sub_pub_for_task(msg.short_id)
        self.__task_manager.receive_task(short_id=msg.short_id, task_desc=msg.data, task_finished_callback=self.__publish_task_finished_info)
        intrest_estimation: IntrestDescription = self.__task_manager.get_intrest(msg.short_id)
        self.__publish_intrest(msg.short_id, intrest_estimation)

    def __create_sub_pub_for_task(self, task_id):
        """ Creates a new topic specific to the newly defined task """
        dynamic_topic_sub_pub = TopicSubPub() 
        dynamic_topic_sub_pub.pub = self.create_publisher(TaskConv, '/mrs_main/id_' + str(task_id), 10)
        dynamic_topic_sub_pub.sub = self.create_subscription(
                                                msg_type=TaskConv,
                                                topic='/mrs_main/id_' + str(task_id),
                                                callback=self.__generic_task_callback,
                                                qos_profile=self._qos_profile
                                            )
        self.task_topic_subpub_dict[task_id] = dynamic_topic_sub_pub

    def __publish_intrest(self, task_id: int, intrest: IntrestDescription):
        """ publishes the intrest estimation regarding the specific task
            (on the topic specific to the task) """
        task_conv_msg = TaskConv()
        task_conv_msg.performative = MrsConvPerform.declare_coord_intrest
        task_conv_msg.data = [str(intrest.coordination)]
        task_conv_msg.short_id = task_id
        task_conv_msg.sender = self.agent_name
        pub: Publisher = self.task_topic_subpub_dict[task_id].pub
        pub.publish(task_conv_msg)

    def __generic_task_callback(self, msg: TaskConv):
        """ Callback run whenever any message shows up on the task-specific topic,
            performs particular actions depending on the current state of the task"""
        if msg.sender != self.agent_name:
            self.get_logger().info(f'I heard msg from {msg.sender}, \
                                   performative: {msg.performative}, task data: {msg.data}')
            conv_msg = TaskConvMsg()
            conv_msg.deserialize(msg=msg)
            answer_msg = self.__task_manager.define_next_behavior(conv_msg)
            if (answer_msg is None): return
            answer_msg.add_conversation_context(sender_name=self.agent_name, id=conv_msg.short_id)
            print(f'[ DEBUG LOG ] Answer msg data {answer_msg.data[0]}')
            conv_answer_msg= answer_msg.serialize()
            self.task_topic_subpub_dict[msg.short_id].pub.publish(conv_answer_msg)

    def __publish_task_finished_info(self):
        pass
        # ros_msg = msg.serialize()
        # pub: Publisher = self.task_topic_subpub_dict[ros_msg.short_id].pub
        # pub.publish(ros_msg)

