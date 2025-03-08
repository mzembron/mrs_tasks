
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
import random

ROOM_DICT = {
    1 : [(-6.0, 1.0), (-6.0, 7.0), (-8.0, 7.0), (-8.0, -1.0), (-6.0, 1.0) ],
    2 : [( -3.0, 1.0), (-3.0, -1.0), (4.0, -1.0), (4.0 , 1.0) ],
    3 : [(-3.0, -4.5), (-8.0, -4.5), (-8.0, -7.5), (-3.0, -7.5), (-3.0, -4.5) ],
    6 : [(-0.5, -4.5), (-0.5, -7.5), (8.5, -7.5), (8.5, -4.5), (-0.5, -4.5) ],
    5:  [(5.0, -2.0), (9.0, -2.0), (9.0, 7.0), (5.0, 7.0), (5.0, -2.0) ],
    4:  [(2.0, 7.0), (-4.0, 7.0), (-4.0, 3.5), (2.0, 3.5) ],
}


class TurtleBot3Navigator(Node):
    def __init__(self, topic_name, agent_name):
        super().__init__('turtlebot3_navigator')
        time.sleep(random.randint(1, 3))
        self.navigator = BasicNavigator(namespace='/' + agent_name)
        self.goal_publisher = self.create_publisher(PoseStamped, topic_name, 10)

    def inspect_room(self, room_number):
        room_points = ROOM_DICT[room_number]
        for point in room_points:
            self.send_goal(point[0], point[1], 0.0)

    def send_goal(self, x, y, yaw):
        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Create a goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = yaw

        # Publish the goal pose to the specified topic
        self.goal_publisher.publish(goal_pose)

        # Send the goal pose
        self.navigator.goToPose(goal_pose)

        # Wait for the result
        # result = self.navigator.getResult()
        while not self.navigator.isTaskComplete():
            print('waiting for goal to be reached...')
            time.sleep(1)
        print('goal achieved')


def main(args=None):
    print('@@@@@@@@@@@ starting separate script @@@@@@@@@@@@@')
    agent_name = ''
    if (len(sys.argv)>1):
        agent_name = sys.argv[1]
        room_number = int(sys.argv[2])
    rclpy.init()
    topic_name = '/' + agent_name + '/goal_pose'
    navigator = TurtleBot3Navigator(topic_name, agent_name)

    try:
        # Example goal coordinates (x, y, yaw)
        navigator.inspect_room(room_number)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()