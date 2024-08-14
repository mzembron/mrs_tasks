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
        msg = TaskConv()
        msg.performative = self.performative
        msg.data = self.data
        msg.short_id = self.short_id
        msg.sender = self.sender
        return msg

    def deserialize(self, msg: TaskConv):
        self.performative: str = msg.performative
        self.data:list[str] = msg.data
        self.short_id: int = msg.short_id
        self.sender: str = msg.sender

    def add_conversation_context(self, sender_name: str, id:int):
        """
        Adds data regarding conversation to the message currently id and agent name
        """
        self.sender = sender_name
        self.short_id = id