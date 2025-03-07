
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import time


class TurtleBot3Navigator(Node):
    def __init__(self, topic_name, agent_name):
        super().__init__('turtlebot3_navigator')
        self.navigator = BasicNavigator(namespace='/' + agent_name)
        self.goal_publisher = self.create_publisher(PoseStamped, topic_name, 10)

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
    agent_name = 'tb2'
    goal_x = 2.0
    goal_y = 0.0
    if (len(sys.argv)>1):
        agent_name = sys.argv[1]
        goal_x = float(sys.argv[2])
        goal_y = float(sys.argv[3])
    rclpy.init()
    topic_name = '/' + agent_name + '/goal_pose'  # Replace with your specific topic name
    navigator = TurtleBot3Navigator(topic_name, agent_name)

    try:
        # Example goal coordinates (x, y, yaw)
        navigator.send_goal(goal_x, goal_y, 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()