from plan_master.turtlebot import Turtlebot

import rospy
from move_base_msgs.msg import MoveBaseActionResult

###  
# Wrapper for robot class with purpose of managing tasks
###

CURRENT_TASK_INDEX = 0

class RobotTaskHarmoniser:
    def __init__(self,robot_name):
        ### just assume we use only turtlebots:
        self.robot = Turtlebot(robot_name)
        rospy.Subscriber(robot_name+"/move_base/result", MoveBaseActionResult, self._action_result_callback)
        self.robot_name = robot_name
        self.task_list = []

    def get_estimated_task_cost(self, goal):
        if self.robot.current_goal is not None:
            #cost from task end position to specif goal
            return (self.robot.calc_cost_from_curr_position_to_curr_goal()
                + self.robot.calc_cost_from_curr_goal_to_spec_position(goal))
        else:
            return self.robot.calc_cost_from_curr_position_to_spec_position(goal)

    def _action_result_callback(self, result):
        print(self.robot.current_goal)
        self.robot.current_goal = None
        
        if(result.status.status == 3):
            print(self.robot_name, " achieved goal! #############")
            self.task_list.pop(CURRENT_TASK_INDEX)
            self.order_task()
        else:
            ### advertise task to plan master 
            print(self.robot_name, " was unable to achive goal! #############")

    def add_task(self, task):
        
        #create task and append but first try:
        if len(self.task_list) == 0:
            self.task_list.append(task)
            self.order_task()
        else:
            self.task_list.append(task) #push front
        
        # print(self.robot_name, " tasks is:", self.task_list)
    
    def order_task(self):
        # print(self.robot_name, " tasks is:", self.task_list)
        if(len(self.task_list) != 0):
            self.robot.go_to_goal(self.task_list[0].data)
