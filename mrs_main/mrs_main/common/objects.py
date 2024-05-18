from rclpy.node import Subscription, Publisher
from mrs_msgs.msg import TaskConv

class IntrestDescription:
    execution: float
    coordination: float


class TopicSubPub:
    sub: Subscription
    pub: Publisher

class TaskConvMsg:
    def __init__(self, msg: TaskConv) -> None:
        self.performative: str = msg.performative
        self.data:list[str] = msg.data
        self.short_id: int = msg.short_id
        self.sender: str = msg.sender
    def serialize(self) -> TaskConv:
        msg = TaskConv()
        msg.performative = self.performative
        msg.data = self.data
        msg.short_id = self.short_id
        msg.sender = self.sender