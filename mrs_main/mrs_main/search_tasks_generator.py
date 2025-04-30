import random
import json

import rclpy
from rclpy.node import Node

from mrs_msgs.msg import TaskDesc
import argparse
import mrs_main.common.constants as mrs_const

ROOM_DICT = {
    1 : [(-6.0, 1.0), (-6.0, 7.0), (-8.0, 7.0), (-8.0, -1.0), (-6.0, 1.0) ],
    2 : [( -3.0, 1.0), (-3.0, -1.0), (4.0, -1.0), (4.0 , 1.0) ],
    3 : [(-3.0, -4.5), (-8.0, -4.5), (-8.0, -7.5), (-3.0, -7.5), (-3.0, -4.5), (-3.0, -7.0)],
    4 : [(-0.5, -4.5), (-0.5, -7.5), (8.5, -7.5), (8.5, -4.5), (-0.5, -4.5) ],
    5:  [(5.0, -2.0), (9.0, -2.0), (9.0, 7.0), (5.0, 7.0), (5.0, -2.0) ],
    6:  [(2.0, 7.0), (-3.0, 7.0), (-3.0, 3.5), (2.0, 3.5), (2.0, 5.0) ],
}

class SearchTasksGenerator(Node):

    def __init__(self):
        super().__init__('dummy_task_generator')
        self.publisher_ = self.create_publisher(TaskDesc, mrs_const.TASKS_DEFINITION_TOPIC_NAME, 10)
        timer_period = 0.5  # seconds
        self.INIT_MSGS = 4
        self.max_messages = self.INIT_MSGS + len(ROOM_DICT)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TaskDesc()
        if self.i < 4:
            msg.type = 'Init'
            msg.short_id = self.i
            msg.data = json.dumps({mrs_const.TASK_DESC_DEPENDENCIES: []})
            self.get_logger().info('Publishing: "%s"' % msg.type)
        elif self.i < self.max_messages:
            room_number = self.i - self.INIT_MSGS + 1
            msg.type = 'Search'
            msg.short_id = self.i
            self.get_logger().info('Publishing: "%s", room: "%s"' % (msg.type,  room_number))
            msg.data = json.dumps({
                mrs_const.TASK_DESC_DEPENDENCIES: self.get_dependencies(room_number),
                mrs_const.SEARCH_WAYPOINTS: ROOM_DICT[room_number],
                                   })
        self.publisher_.publish(msg)
        self.i += 1
        if(self.i > self.max_messages): # max int8 value
            self.get_logger().info('Shutting down')
            self.destroy_timer(self.timer)
            self.destroy_node()
            rclpy.shutdown()

    def get_dependencies(self, task_number: int) -> list[int]:
        if task_number > 6 and task_number < 9:
            return [4, 5, 6]
        else:
            return []


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = SearchTasksGenerator()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()