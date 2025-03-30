import rclpy
import time
import logging
import json
import mrs_main.common.constants as mrs_const

from rclpy.node import Node, Publisher
from mrs_msgs.msg import TaskDesc, TaskConv, TaskBacklog, TasksStatesDeclaration, TaskAlign, TaskUpdate 
from mrs_main.tasks_management.task_manager import TaskManager
from mrs_main.common.objects import IntrestDescription, TopicSubPub, TaskConvMsg, TaskData
from mrs_main.common.conversation_data import MrsConvPerform

# agent type specific
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, ReliabilityPolicy

logger = logging.getLogger(__name__)

class OrdersManager(Node):
    """ Orders Manager takes care of communication in the contexts of 
    diffrent tasks - every task has its own ROS topic.
    """
    def __init__(self, agent_name: str, task_manager: TaskManager):
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
        self._qos_profile = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription_task_def_topic = self.create_subscription(
            msg_type=TaskDesc,
            topic=mrs_const.TASKS_DEFINITION_TOPIC_NAME,
            callback=self.__task_definition_callback,
            qos_profile=self._qos_profile
        )
        self._tasks_states_publisher = self.create_publisher(TasksStatesDeclaration, '/mrs_main/tasks_states_declaration', qos_profile=self._qos_profile)
        self._backlog_info_publisher = self.create_publisher(TaskBacklog, '/mrs_main/backlog_updates', qos_profile=self._qos_profile)
        self._align_publisher = self.create_publisher(TaskAlign, '/mrs_main/tasks_align', qos_profile=self._qos_profile)
        self._backlog_info_subscription = self.create_subscription(
            msg_type=TaskAlign,
            topic='/mrs_main/tasks_align',
            callback=self.__align_tasks,
            qos_profile=self._qos_profile
        )

        self.create_subscription(msg_type=Odometry, topic='/' + self.agent_name+'/odom', callback=self.__update_knowledge_base, qos_profile =10)
        self.task_topic_subpub_dict: dict[int, TopicSubPub] = {} 

        self.__task_manager = task_manager

        self.create_timer(1.0, self.__publish_tasks_states_decalration) # utilize ros to publish backlog info
        self.create_timer(1.0, self.__publish_backlog_info)

    def __task_definition_callback(self, msg: TaskDesc):
        """ Callback for the generic topicwith defintion of any task (action entrypoint)"""
        self.get_logger().info(f'I heard task: {msg.type}')
        
        self.__create_sub_pub_for_task(msg.short_id)
        task_data = TaskData.from_task_definition(msg.short_id, msg.data)
        task_desc = msg.data
        intrest_estimation: IntrestDescription = self.__task_manager.receive_task(short_id=msg.short_id, task_desc=task_desc , task_data=task_data, task_finished_callback=self.__publish_task_finished_info, orders_manager=self)
        # intrest_estimation: IntrestDescription = self.__task_manager.get_intrest(msg.short_id)
        # time.sleep(3) # wait for others tio create theirs publishers
        self.__publish_intrest(msg.short_id, intrest_estimation)

    def __create_sub_pub_for_task(self, task_id):
        """ Creates a new topic specific to the newly defined task """
        dynamic_topic_sub_pub = TopicSubPub() 
        dynamic_topic_sub_pub.pub = self.create_publisher(TaskConv, '/mrs_main/id_' + str(task_id), qos_profile=self._qos_profile)
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
            self.get_logger().info(f'I heard msg from {msg.sender} about task {msg.short_id}, \
                                   performative: {msg.performative}, task data: {msg.data}')
            conv_msg = TaskConvMsg()
            conv_msg.deserialize(msg=msg)
            answer_msg = self.__task_manager.define_next_behavior(conv_msg)
            if (answer_msg is None): return
            answer_msg.add_conversation_context(sender_name=self.agent_name, id=conv_msg.short_id)
            print(f'[ DEBUG LOG ] Answer msg data {answer_msg.data[0]}')
            conv_answer_msg= answer_msg.serialize()
            self.task_topic_subpub_dict[msg.short_id].pub.publish(conv_answer_msg)

    def __publish_task_finished_info(self, task_data: TaskData):
        #   TODO: refine this method: maybe it should be a generic callback to publish TaskConv from
        #      the task manager
        ros_msg: TaskConv = task_data.serialize(MrsConvPerform.inform_task_finished, self.agent_name)
        pub: Publisher = self.task_topic_subpub_dict[ros_msg.short_id].pub
        pub.publish(ros_msg)

    def __publish_backlog_info(self):
        # 1: self.__task_manager.task_dict - make retreaving thread safe
        # 2: dump task dict into TaskBacklog
        # 3: publish
        states_info =self.__task_manager.get_states_list()
        states_data = self.__task_manager.get_states_data()
        task_desc_list = self.__task_manager.get_task_desc_list()
        align_msg = TaskAlign() # for now just empty msg
        align_msg.sender = self.agent_name
        for idx, state in enumerate(states_info):
            if state == 'init':
                continue
            update_msg = TaskUpdate()
            
            update_msg.task_state = state
            update_msg.task_desc.short_id = idx
            update_msg.task_desc.data = task_desc_list[idx] # TODO: naming misconception, to be fixed
            update_msg.task_conv_data = json.dumps(states_data[idx])
            align_msg.update_list.append(update_msg)
        self._align_publisher.publish(align_msg)

    def __publish_tasks_states_decalration(self):
        msg = TasksStatesDeclaration()
        msg.robot_name = self.agent_name
        msg.tasks_states = self.__task_manager.get_states_list()
        self._tasks_states_publisher.publish(msg)

    def __align_tasks(self, align_msg: TaskAlign):
        # task manager compare backlog
        if align_msg.sender == self.agent_name:
            return
        
        states_info =self.__task_manager.get_states_list()
        for task_align in align_msg.update_list:
            align_task_state = task_align.task_state
            curr_task_state = states_info[task_align.task_desc.short_id]
            if curr_task_state == 'init' and align_task_state != 'init':
                logger.warning(f'Found mismatch at the definition of tasks for task id: {task_align.task_desc.short_id}, curr state: init, align state: {align_task_state}')
                logger.info(' -------------------- initializing task from task assignment --------------------')
                self.__create_sub_pub_for_task(task_align.task_desc.short_id)
                task_data = TaskData.from_task_definition(task_align.task_desc.short_id, task_align.task_desc.data)
                intrest_estimation: IntrestDescription = self.__task_manager.receive_task(short_id=task_align.task_desc.short_id, task_desc=task_align.task_desc.data, task_data=task_data, task_finished_callback=self.__publish_task_finished_info, orders_manager=self)

    def __update_knowledge_base(self, msg: Odometry):
        self.__task_manager._knowledge_base.update_position(msg.pose.pose.position.x, msg.pose.pose.position.y)

