import random
import json

import rclpy
from rclpy.node import Node

from mrs_msgs.msg import TaskDesc
import argparse
import mrs_main.common.constants as mrs_const

class SingleTaskPublisher(Node):

    def __init__(self, short_id: int):
        super().__init__('single_task_publisher')
        self.publisher_ = self.create_publisher(TaskDesc, mrs_const.TASKS_DEFINITION_TOPIC_NAME, 10)
        self.short_id = short_id
        self.send = False
        self.timer = self.create_timer(1, self.publish_task)

    def publish_task(self):
        if self.send:
            return
        self.send = True
        msg = TaskDesc()
        msg.type = 'Search'
        msg.short_id = self.short_id
        msg.data = json.dumps({mrs_const.TASK_DESC_DEPENDENCIES: []})
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.type)
        self.get_logger().info('Id: "%s"' % msg.short_id)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Single Task Publisher')
    parser.add_argument('--short_id', type=int, required=True, help='Short ID of the task to publish')
    args = parser.parse_args()
    short_id = args.short_id
    single_publisher = SingleTaskPublisher(short_id)

    rclpy.spin(single_publisher)

    single_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()