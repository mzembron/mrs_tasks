import math

import rospy
from nav_msgs.msg import  Odometry
from move_base_msgs.msg import MoveBaseActionGoal
# from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.srv import GetPlan, GetPlanRequest

from helpful_functions.data_manipulation import DataManipulation
from constants.robot_type import RobotType
from constants.constants import ROBOT_AVERAGE_VELOCITY
import math

class Turtlebot:

    def __init__(self, robot_name):
        rospy.Subscriber(robot_name + "/odom", Odometry, self._odom_callback)
        self.goal_publisher = rospy.Publisher(robot_name + "/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.make_plan_srv = rospy.ServiceProxy(robot_name + "/move_base_node/NavfnROS/make_plan", GetPlan)
        self.robot_name = robot_name
        self.robot_type = RobotType.MOBILE
        self.current_goal = None
    
    def calc_cost_from_curr_position_to_spec_position(self, goal_odom_data):
        plan = self._make_path_plan(self.current_odom_data.pose, goal_odom_data)
        return self._approximate_path_length(plan)/ROBOT_AVERAGE_VELOCITY

    def calc_cost_from_curr_position_to_curr_goal(self):
        assert(self.current_goal is not None)

        plan = self._make_path_plan(self.current_odom_data.pose, self.current_goal)
        return self._approximate_path_length(plan)/ROBOT_AVERAGE_VELOCITY

    def calc_cost_from_spec_position_to_spec_position(self, start_odom_data, goal_odom_data):
        plan = self._make_path_plan(start_odom_data, goal_odom_data)
        return self._approximate_path_length(plan)/ROBOT_AVERAGE_VELOCITY

    def calc_cost_from_curr_goal_to_spec_position(self, goal_odom_data):
        assert(self.current_goal is not None)

        plan = self._make_path_plan(self.current_goal, goal_odom_data)
        return self._approximate_path_length(plan)/ROBOT_AVERAGE_VELOCITY

    def go_to_goal(self, goal_odom_data):
        msg = MoveBaseActionGoal()
        DataManipulation.copy_pose_data(msg.goal.target_pose, goal_odom_data)
        self.current_goal = goal_odom_data
        self.goal_publisher.publish(msg)

    def _odom_callback(self, data):
        self.current_odom_data = data


    def _make_path_plan(self, start_odom_data, goal_odom_data):
        req = GetPlanRequest()

        DataManipulation.copy_pose_data(req.start, start_odom_data)
        DataManipulation.copy_pose_data(req.goal, goal_odom_data)

        req.tolerance = 0.1
        service_output = self.make_plan_srv(req)
        return  service_output.plan

    def _approximate_path_length(self, plan):
        path_length = 0.0

        for index in range(1, len(plan.poses) -1):
            previous = plan.poses[index -1].pose.position
            current = plan.poses[index].pose.position
            #  Euclidean distance
            distance = math.sqrt((previous.x - current.x)**2 + (previous.y - current.y)**2)
            path_length += distance

        return path_length






