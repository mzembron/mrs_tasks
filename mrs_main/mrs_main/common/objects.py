from rclpy.node import Subscription, Publisher


class IntrestDescription:
    execution: float
    coordination: float


class TopicSubPub:
    sub: Subscription
    pub: Publisher