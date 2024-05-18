import rclpy
from rclpy.node import Node
from mrs_msgs.msg import TaskDesc
import common.constants as mrs_const

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TaskDesc, mrs_const.TASKS_DEFINITION_TOPIC_NAME, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TaskDesc()
        msg.type = 'Hello World: %d' % self.i
        msg.short_id = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.type)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()