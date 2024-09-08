import random
import json

import rclpy
from rclpy.node import Node

from mrs_msgs.msg import TaskDesc
import mrs_main.common.constants as mrs_const

class DummyTaskGenerator(Node):

    def __init__(self):
        super().__init__('dummy_task_generator')
        self.publisher_ = self.create_publisher(TaskDesc, mrs_const.TASKS_DEFINITION_TOPIC_NAME, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TaskDesc()
        msg.type = 'Hello World: %d' % self.i
        msg.short_id = self.i
        msg.data = json.dumps({'dependencies': self.genearate_dependencies(self.i)})
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.type)
        self.i += 1

    def genearate_dependencies(self, task_number: int) -> list[int]:
        if task_number <= 5:
            return []

        return random.sample(range(task_number), 2)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DummyTaskGenerator()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()