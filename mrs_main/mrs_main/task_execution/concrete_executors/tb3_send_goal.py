
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class TurtleBot3Navigator(Node):
    def __init__(self, topic_name):
        super().__init__('turtlebot3_navigator')
        self.navigator = BasicNavigator(namespace='/tb2')
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
        result = self.navigator.getResult()

        # # Check the result
        # if result == NavigationResult.SUCCEEDED:
        #     self.get_logger().info('Goal reached successfully!')
        # else:
        #     self.get_logger().info('Failed to reach the goal.')

def main(args=None):
    
    rclpy.init()
    agent_name = 'tb2'
    topic_name = '/' + agent_name + '/goal_pose'  # Replace with your specific topic name
    navigator = TurtleBot3Navigator(topic_name)

    try:
        # Example goal coordinates (x, y, yaw)
        navigator.send_goal(0.0, 1.0, 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()