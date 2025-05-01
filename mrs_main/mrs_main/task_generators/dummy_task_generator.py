import random
import json

import rclpy
from rclpy.node import Node

from mrs_msgs.msg import TaskDesc
import argparse
import mrs_main.common.constants as mrs_const

class DummyTaskGenerator(Node):

    def __init__(self, max_messages=127):
        super().__init__('dummy_task_generator')
        self.publisher_ = self.create_publisher(TaskDesc, mrs_const.TASKS_DEFINITION_TOPIC_NAME, 10)
        timer_period = 1  # seconds
        self.max_messages = max_messages
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        msg = TaskDesc()
        msg.type = 'Search' 
        msg.short_id = self.i
        msg.data = json.dumps({mrs_const.TASK_DESC_DEPENDENCIES: self.genearate_dependencies(self.i)})
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.type)
        self.get_logger().info('Id: "%s"' % msg.short_id)
        self.i += 1
        if(self.i > self.max_messages): # max int8 value
            self.get_logger().info('Shutting down')
            self.destroy_timer(self.timer)
            self.destroy_node()
            rclpy.shutdown()

    def genearate_dependencies(self, task_number: int) -> list[int]:
        if task_number == 7:
            return [4]
        if task_number == 8:
            return [4]
        if task_number == 9:
            return [4]
        if task_number <= 30:
            return []

        return random.sample(range(task_number), 2)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Dummy Task Generator')
    parser.add_argument('--max', type=int, default=127, help='Maximum number of messages to publish')
    args = parser.parse_args()
    max_messages = args.max
    minimal_publisher = DummyTaskGenerator(max_messages)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()