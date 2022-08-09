import math

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import  Odometry
from geometry_msgs.msg import PoseStamped

from Turtlebot import Turtlebot
from HelpfulFunctions import HelpfulFunctions


class PlanMaster():
    def __init__(self):
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._move_base_goal_callback)
        self.robots = []

    def subscribe(self, robot):
        self.robots.append(robot)
    def _move_base_goal_callback(self, data):
        # msg = MoveBaseActionGoal()
        # HelpfulFunctions.copy_pose_data(msg.goal.target_pose, data)
        # print(self.robots[0].get_operation_cost(data))
        optimal_robot = self._select_optimal_robot(data)
        optimal_robot.go_to_goal(data)


    def _select_optimal_robot(self, operation_data):
        robots_cost_dict = {}
        for robot in self.robots:
            print("[ Estimated cost for: ", robot.robot_name, "] ", robot.get_operation_cost(operation_data))
            robots_cost_dict[robot] = robot.get_operation_cost(operation_data)
        
        # robots_cost_dict = sorted(robots_cost_dict.items(), key=lambda x: x[1])
        return min(robots_cost_dict, key = robots_cost_dict.get)
        # return
    
        