import rclpy
from rclpy.node import Node
from mrs_msgs.msg import TaskDesc, TaskConv, TaskAlign
import csv
import signal
import sys
from datetime import datetime


class MsgTimelineRecorder(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('msg_timeline_recorder')

        # List of topics to subscribe to with their respective message types
        self.topics = {
            '/mrs_tasks/task_definition': TaskDesc,
            '/mrs_main/tasks_align': TaskAlign,
            '/mrs_main/id_4': TaskConv,
            '/mrs_main/id_5': TaskConv,
            '/mrs_main/id_6': TaskConv,
            '/mrs_main/id_7': TaskConv,
            '/mrs_main/id_8': TaskConv,
            '/mrs_main/id_9': TaskConv,
            '/mrs_main/id_10': TaskConv,
        }

        # Array to store messages and timestamps
        self.recorded_data = []

        # Subscribe to all topics
        for topic, msg_type in self.topics.items():
            self.create_subscription(msg_type, topic, lambda msg, t=topic: self.callback(msg, t), 1000)

        # Handle shutdown gracefully
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def callback(self, msg, topic):
        """Callback function to handle incoming messages."""
        timestamp = self.get_clock().now().to_msg()
        formatted_time = datetime.fromtimestamp(timestamp.sec + timestamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M:%S.%f')
        self.recorded_data.append({
            'topic': topic,
            'timestamp': formatted_time,
            # 'message': str(msg)
        })
        self.get_logger().info(f"Recorded message from {topic} at {formatted_time}")

    def shutdown_handler(self, signum, frame):
        """Handle shutdown and write data to CSV."""
        self.get_logger().info("Shutting down and writing data to CSV...")
        self.write_to_csv()
        rclpy.shutdown()
        sys.exit(0)

    def write_to_csv(self):
        """Write recorded data to a CSV file."""
        filename = 'msg_timeline.csv'
        with open(filename, mode='w', newline='') as csvfile:
            fieldnames = ['topic', 'timestamp', 'message']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for entry in self.recorded_data:
                writer.writerow(entry)

        self.get_logger().info(f"Data written to {filename}")


def main(args=None):
    rclpy.init(args=args)
    recorder = MsgTimelineRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        # recorder.shutdown_handler(None, None)
        recorder.write_to_csv()
        raise KeyboardInterrupt


if __name__ == '__main__':
    main()