import threading


import rospy
import time

from plan_master.turtlebot import Turtlebot

from move_base_msgs.msg import MoveBaseActionResult

from mrs_msgs.msg import TaskBacklog

###  
# Wrapper for robot class with purpose of managing tasks
###

CURRENT_TASK_INDEX = 0
DELAY_TASK_PENALTY = 4


class RobotTaskHarmoniser:
    def __init__(self,robot_name):
        ### just assume we use only turtlebots:
        self.robot = Turtlebot(robot_name)
        rospy.Subscriber(robot_name+"/move_base/result", MoveBaseActionResult, self._action_result_callback)
        self.tasks_backlog_publisher = rospy.Publisher("plan_master/tasks_backlog", TaskBacklog, queue_size=10)
        self.backlog_publishing_thread = threading.Thread(target=self.backlog_updater, daemon=True)
        self.backlog_publishing_thread.start()
        self.robot_name = robot_name
        self.task_list = []

    def __del__(self):
        self.backlog_publishing_thread.join()
        

    def get_estimated_task_cost(self, new_task):
        full_cost = 0 
        was_new_task_counted = False
        if len(self.task_list) == 0:
            return self.robot.calc_cost_from_curr_position_to_spec_position(new_task.data)

        for task_index in range(0, len(self.task_list)):
            task = self.task_list[task_index]
            task_dealy_coeficient = self._calculate_delay_coeficient(task, new_task)

            if task_index == 0:
                full_cost += self.robot.calc_cost_from_curr_position_to_spec_position(task.data)/task_dealy_coeficient

            elif task_dealy_coeficient == DELAY_TASK_PENALTY and not was_new_task_counted:
                # including new task into estimated cost 
                previous_task =  self.task_list[task_index-1]
                full_cost += self.robot.calc_cost_from_spec_position_to_spec_position(previous_task.data, new_task.data)/task_dealy_coeficient
                was_new_task_counted = True
                task_dealy_coeficient = DELAY_TASK_PENALTY
                full_cost += self.robot.calc_cost_from_spec_position_to_spec_position(new_task.data, task.data)/task_dealy_coeficient
            
            else:
                previous_task =  self.task_list[task_index-1]
                full_cost += self.robot.calc_cost_from_spec_position_to_spec_position(previous_task.data, task.data)/task_dealy_coeficient
        
        if was_new_task_counted is False:
            last_task = self.task_list[-1]
            full_cost += self.robot.calc_cost_from_spec_position_to_spec_position(last_task.data, new_task.data)

        print(self.robot_name, full_cost)
        return full_cost 


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

    def backlog_updater(self):
        while(not rospy.is_shutdown()):
            time.sleep(5)
            backlog = TaskBacklog()
            backlog.robot_name = self.robot_name
            for task in self.task_list:
                backlog.tasks.append(task.return_task_desc_msg())
            
            self.tasks_backlog_publisher.publish(backlog)
    
    def _calculate_delay_coeficient(self, curr_task, new_task):
        task_dealy_coeficient = 1
        if new_task.priority > curr_task.priority:
            task_dealy_coeficient = DELAY_TASK_PENALTY
        return task_dealy_coeficient

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
