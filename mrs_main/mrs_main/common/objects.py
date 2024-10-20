from rclpy.node import Subscription, Publisher
from mrs_msgs.msg import TaskConv

class IntrestDescription:
    execution: float
    coordination: float


class TopicSubPub:
    sub: Subscription
    pub: Publisher

class TaskConvMsg:

    def serialize(self) -> TaskConv:
        """" Serializes the message to the defined ROS2 message type"""
        msg = TaskConv()
        msg.performative = self.performative
        msg.data = self.data
        msg.short_id = self.short_id
        msg.sender = self.sender
        return msg

    def deserialize(self, msg: TaskConv):
        """Deserializes the ROS2 message to the object attributes"""
        self.performative: str = msg.performative
        self.data:list[str] = msg.data
        self.short_id: int = msg.short_id
        self.sender: str = msg.sender

    def add_conversation_context(self, sender_name: str, id:int):
        """
        Adds data regarding conversation to the message,
            currently: 
                - id
                - agent name
        """
        self.sender = sender_name
        self.short_id = id

class TaskData:
    """ Class holds base info regarding task """
    def __init__(self, task_conv_msg: TaskConvMsg):
        self.task_data = task_conv_msg