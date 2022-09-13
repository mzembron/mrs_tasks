import math

import rospy
from nav_msgs.msg import  Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
# from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.srv import GetPlan, GetPlanRequest

from HelpfulFunctions import HelpfulFunctions
import math

LATEST_STATUS_INDEX = 0
PENDING = 0
ACTIVE = 1

class Turtlebot:

    def __init__(self, robot_name):
        rospy.Subscriber(robot_name + "/odom", Odometry, self._odom_callback)
        rospy.Subscriber(robot_name+"/move_base/result", MoveBaseActionResult, self._action_result_callback)
        self.goal_publisher = rospy.Publisher(robot_name + "/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.make_plan_srv = rospy.ServiceProxy(robot_name + "/move_base_node/NavfnROS/make_plan", GetPlan)
        self.robot_name = robot_name
        self.current_goal = None
        # self.current_goal_status = None - 
    
    def get_operation_cost_from_current_position(self, goal_odom_data):
        plan = self._make_path_plan(self.current_odom_data.pose, goal_odom_data)
        return self._approximate_path_length(plan)

    def get_operation_cost_fro_specific_position(self, start_odom_data, goal_odom_data):
        plan = self._make_path_plan(start_odom_data, goal_odom_data)
        return self._approximate_path_length(plan)

    
       
    
    def go_to_goal(self, goal_odom_data):
        msg = MoveBaseActionGoal()
        HelpfulFunctions.copy_pose_data(msg.goal.target_pose, goal_odom_data)
        self.current_goal = goal_odom_data
        self.goal_publisher.publish(msg)

    def _odom_callback(self, data):
        self.current_odom_data = data

        # DEBUG!
        # print(self.robot_name, "Current_goal: ", self.current_goal)

    # def _task_status_callback(self, status):
    #     #TODO! 
    #     ## problem with synchronizing provided 
    #     # pass
    #     # Not really working
    #     print(self.robot_name)
    #     print(status.status_list)
    #     # self.current_goal_status = status.status_list[LATEST_STATUS_INDEX].status
    #     # if status.status_list[LATEST_STATUS_INDEX].status is not (PENDING or ACTIVE):
    #     #     self.current_goal = None

    def _action_result_callback(self, result):
        print(self.current_goal)
        self.current_goal = None
        
        if(result.status.status == 3):
            print(self.robot_name, " achieved goal! #############")
        else:
            print(self.robot_name, " was unable to achive goal! #############")


    def _make_path_plan(self, start_odom_data, goal_odom_data):
        req = GetPlanRequest()

        HelpfulFunctions.copy_pose_data(req.start, start_odom_data)
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






