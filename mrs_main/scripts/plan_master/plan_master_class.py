import math

import rospy
from geometry_msgs.msg import PoseStamped


from plan_master.task.task import Task
from mrs_msgs.msg import TaskDesc
from data_parser.room_coordinates_parser import RoomCoordinatesParser


class PlanMaster():
    def __init__(self):
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._move_base_goal_callback)
        rospy.Subscriber("plan_master/order_task", TaskDesc, self._order_task_callback)
        #subscription of task on general topic like /plan_master/ordered_tasks
        self.robots = []

    def subscribe(self, robot):
        self.robots.append(robot)

    def _move_base_goal_callback(self, data):
        task = Task('GT', data, priority= 8)
        self._order_task(task)

    def _order_task_callback(self, task_desc):
        try:
            room_coordinates = RoomCoordinatesParser().get_room_coordinates(task_desc.data)
            task_data = self._prepare_task_data(room_coordinates)
            task = Task('GT', task_data, priority = task_desc.priority)
            self._order_task(task)
        except(KeyError):
            print("Room name does not exist in system!")

    def _order_task(self, task):
        optimal_robot, position = self._select_optimal_robot(task)
        optimal_robot.add_task(task, position)

    def _select_optimal_robot(self, task):
        robots_cost_dict = {}
        robots_task_position_dict = {}
        for robot in self.robots:
            robots_cost_dict[robot], robots_task_position_dict[robot] = robot.get_estimated_task_cost_with_scheduled_position(task)
            print("[ Estimated cost for: ", robot.robot_name, "] ", robots_cost_dict[robot], 
                "Task at position: ", robots_task_position_dict[robot])
            
        
        # robots_cost_dict = sorted(robots_cost_dict.items(), key=lambda x: x[1])
        optimal_robot = min(robots_cost_dict, key = robots_cost_dict.get) 
        return optimal_robot, robots_task_position_dict[optimal_robot]
        # return

    def _prepare_task_data(self, coordinates):
        task_data = PoseStamped()
        task_data.pose.position.x = coordinates[0]
        task_data.pose.position.y = coordinates[1]
        task_data.pose.position.z = 0
        
        task_data.pose.orientation.x = 0
        task_data.pose.orientation.y = 0 
        task_data.pose.orientation.z = 0 
        task_data.pose.orientation.w = 1 
        return task_data

