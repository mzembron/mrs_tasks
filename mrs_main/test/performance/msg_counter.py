import rclpy
from rclpy.node import Node
from mrs_msgs.msg import  TaskConv
from rclpy.qos import QoSProfile, ReliabilityPolicy


class MessageCounter(Node):

    def __init__(self, topic_name):
        super().__init__('message_counter')
        self._qos_profile = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            TaskConv,  # Change this to the appropriate message type for your topic
            topic_name,
            self.listener_callback,
            qos_profile=self._qos_profile)
        self.subscription  # prevent unused variable warning
        self.message_count = 0

    def listener_callback(self, msg):
        self.message_count += 1

    def get_message_count(self):
        return self.message_count

def main(args=None):
    rclpy.init(args=args)
    topic_name = '/mrs_main/id_0'  # Replace with your topic name
    message_counter = MessageCounter(topic_name)

    try:
        rclpy.spin(message_counter)
    except KeyboardInterrupt:
        message_counter.get_logger().info('Shutting down message counter...')
    finally:
        message_counter.get_logger().info(f'Total messages received: {message_counter.get_message_count()}')
        message_counter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()