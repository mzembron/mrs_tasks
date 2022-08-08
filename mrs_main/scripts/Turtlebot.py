import math

import rospy
from nav_msgs.msg import  Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.srv import GetPlan, GetPlanRequest

from HelpfulFunctions import HelpfulFunctions
import math

class Turtlebot:

    def __init__(self, odometry_sub_topic_name, goal_pub_topic_name):
        self.rospy = rospy
        self.rospy.Subscriber(odometry_sub_topic_name, Odometry, self._odom_callback)
        self.goal_publisher = rospy.Publisher(goal_pub_topic_name, MoveBaseActionGoal, queue_size=10)
        self.make_plan_srv = rospy.ServiceProxy('/robot1/move_base_node/NavfnROS/make_plan', GetPlan)
    
    def get_operation_cost(self, goal_odom_data):
        plan = self._make_path_plan(goal_odom_data)
        #DEBUG!
        # check:
        # http://docs.ros.org/en/diamondback/api/nav_msgs/html/classnav__msgs_1_1srv_1_1__GetPlan_1_1GetPlanResponse.html
        # print(type(plan.plan.poses))
        # for element in plan.poses:
        #     print(element.pose.position)

        # print(self._approximate_path_length(plan))
        return self._approximate_path_length(plan)
       
    
    def go_to_goal(self, goal_odom_data):
        msg = MoveBaseActionGoal()
        HelpfulFunctions.copy_pose_data(msg.goal.target_pose, goal_odom_data)

    def _odom_callback(self, data):
        self.current_odom_data = data

    def _make_path_plan(self, goal_odom_data):
        req = GetPlanRequest()

        HelpfulFunctions.copy_pose_data(req.start, self.current_odom_data.pose)
        HelpfulFunctions.copy_pose_data(req.goal, goal_odom_data)

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






