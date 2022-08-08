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
        self.robots.append(Turtlebot("/robot1/odom", '/robot1/move_base/goal'))

    def _move_base_goal_callback(self, data):
        # msg = MoveBaseActionGoal()
        # HelpfulFunctions.copy_pose_data(msg.goal.target_pose, data)
        self.robots[0].get_operation_cost(data)
        