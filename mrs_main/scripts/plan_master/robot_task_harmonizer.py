import threading

import rospy
import time
from constants.constants import *

from plan_master.turtlebot import Turtlebot

from move_base_msgs.msg import MoveBaseActionResult

from mrs_msgs.msg import TaskBacklog


CURRENT_TASK_INDEX = 0


class RobotTaskHarmonizer:
    """ Wrapper for robot class with purpose of managing tasks """
    def __init__(self, robot_name):
        """
        Just assume we use only turtlebots:"""
        self.robot = Turtlebot(robot_name)
        rospy.Subscriber(robot_name+"/move_base/result", MoveBaseActionResult, self._action_result_callback)
        self.tasks_backlog_publisher = rospy.Publisher("plan_master/tasks_backlog", TaskBacklog, queue_size=10)
        self.backlog_publishing_thread = threading.Thread(target=self.backlog_updater, daemon=True)
        self.backlog_publishing_thread.start()
        self.robot_name = robot_name
        self.task_list = []

    def __del__(self):
        self.backlog_publishing_thread.join()

    def get_estimated_task_cost_with_scheduled_position(self, new_task):
        full_cost = 0 
        was_new_task_included = False
        task_position = -1 # initial position is last in task list
        if len(self.task_list) == 0:
            task_position = 0
            return self.robot.calc_cost_from_curr_position_to_spec_position(new_task.data), task_position

        for task_index in range(0, len(self.task_list)):
            task = self.task_list[task_index]
            task_dealy_coeficient = self._calculate_delay_coeficient(task, new_task)

            if task_index == 0:
                full_cost += self.robot.calc_cost_from_curr_position_to_spec_position(task.data)/task_dealy_coeficient

            elif (new_task.has_higher_priority_than(task)) and not was_new_task_included:
                # if new task has higher priority than current task including new task into estimated cost
                task_position = task_index
                full_cost += self._calculate_cost_of_including_new_task(task_index, new_task)
                was_new_task_included = True

            else:
                previous_task =  self.task_list[task_index-1]
                full_cost += self.robot.calc_cost_from_spec_position_to_spec_position(previous_task.data, task.data)/task_dealy_coeficient
        
        if was_new_task_included is False:
            last_task = self.task_list[-1]
            full_cost += self.robot.calc_cost_from_spec_position_to_spec_position(last_task.data, new_task.data)

        print(self.robot_name, full_cost)
        return full_cost, task_position 


    def add_task(self, task, position):
        if len(self.task_list) == 0:
            self.task_list.append(task)
            self.order_task()
        elif position == -1:
            self.task_list.append(task)
        else:
            self.task_list.insert(position, task)
    
    def order_task(self):
        if(len(self.task_list) != 0):
            self.robot.go_to_goal(self.task_list[0].data)

    def backlog_updater(self):
        while(not rospy.is_shutdown()):
            time.sleep(5)
            backlog = TaskBacklog()
            backlog.robot_name = self.robot_name
            for task in self.task_list:
                backlog.tasks.append(task.return_task_desc_msg())
            
            self.tasks_backlog_publisher.publish(backlog)

    def _calculate_cost_of_including_new_task(self, curr_task_index, new_task):
        cost = 0
        previous_task =  self.task_list[curr_task_index-1]
        curr_task = self.task_list[curr_task_index]
        cost += self.robot.calc_cost_from_spec_position_to_spec_position(previous_task.data, new_task.data)
        cost += self.robot.calc_cost_from_spec_position_to_spec_position(new_task.data, curr_task.data)/DELAY_TASK_PENALTY
        return cost

    def _calculate_delay_coeficient(self, curr_task, new_task):
        task_dealy_coeficient = 1
        if new_task.priority < curr_task.priority:  # lower number highier priority
            task_dealy_coeficient = DELAY_TASK_PENALTY
        return task_dealy_coeficient

    def _action_result_callback(self, result):
        print(self.robot.current_goal)
        self.robot.current_goal = None
        
        if(result.status.status == 3):
            print(self.robot_name, " achieved goal! #############")
            if(len(self.task_list) != 0):
                self.task_list.pop(CURRENT_TASK_INDEX)
                self.order_task()
        else:
            ### advertise task to plan master 
            print(self.robot_name, " was unable to achive goal! #############")

    def _was_task_delayed(self, task_dealy_coeficient):
        return task_dealy_coeficient == DELAY_TASK_PENALTY
