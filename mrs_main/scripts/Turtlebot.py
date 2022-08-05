import math

import rospy
from nav_msgs.msg import  Odometry
from move_base_msgs.msg import MoveBaseActionGoal

class Turtlebot:

    def __init__(self, odometry_sub_topic_name, goal_pub_topic_name):
        self.rospy = rospy
        self.rospy.Subscriber(odometry_sub_topic_name, Odometry, self._odom_callback)
        self.goal_publisher = rospy.Publisher(goal_pub_topic_name, MoveBaseActionGoal, queue_size=10)
    
    def getOperationCost(self, goal_odom_data):
        x_goal = goal_odom_data.pose.position.x
        y_goal = goal_odom_data.pose.position.y

        x_current = self.current_odom_data.pose.pose.position.x -0.5
        y_current = self.current_odom_data.pose.pose.position.y -0.5

        distance =  math.sqrt((x_current-x_goal)**2 + (y_current-y_goal)**2)
        return distance
    
    def go_to_goal(self, goal_odom):
        msg = MoveBaseActionGoal()
        msg.goal.target_pose.header.frame_id = 'map'
        msg.goal.target_pose.pose.position.x = goal_odom.pose.position.x
        msg.goal.target_pose.pose.position.y = goal_odom.pose.position.y

        msg.goal.target_pose.pose.orientation.x = goal_odom.pose.orientation.x
        msg.goal.target_pose.pose.orientation.y = goal_odom.pose.orientation.y
        msg.goal.target_pose.pose.orientation.z = goal_odom.pose.orientation.z
        msg.goal.target_pose.pose.orientation.w = goal_odom.pose.orientation.w

    def _odom_callback(self, data):
        self.current_odom_data = data
        # print("[ Robot ]Odom position x: " + str(data.pose.pose.position.x -0.5))
        # print("[ Robot ]Odom position y: " + str(data.pose.pose.position.y - 0.5))


