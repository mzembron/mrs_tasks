from mrs_msgs.msg import TaskDesc
from constants.robots_usecases import *
from constants.scenario_list import SCENARIO_LIST
###
# Task class
# priority:
#   int from 0 to 10
#   0 - most important 
#   10 - least important
###

class Task():
    
    def __init__(self, type, data, priority = 10):
        self.type = type
        
        self.priority = priority
        self.data = data
        self.is_scenario = self._determine_if_task_is_scenario()
        if self.is_scenario:
            #TODO - how to split task data
            self._generate_subtasks()

    def parse_task_from_msg(self, msg):
        self.type = msg.type
        # self.data = None
        self.priority = msg.prioirty

    def return_task_desc_msg(self):
        #Debug!
        # self.data = None
        task_msg = TaskDesc()
        task_msg.data = "x: "+str(self.data.pose.position.x) + "y: " + str(self.data.pose.position.y)
        task_msg.priority = self.priority
        task_msg.type = self.type
        return task_msg
    
    def is_suitable_for_robot(self, robot):
        return self.type in ROBOT_USECASE_MAP[robot.robot_type]

    def has_highier_priorty_than(self, other_task):
        return self.priority < other_task.priority

    def _determine_if_task_is_scenario(self):
        return self.type in SCENARIO_LIST

    def _generate_subtasks(self):
        self.subtasks_list = []
        for task_type in SCENARIO_LIST[self.type]["subtasks"]:
            subtask = Task(task_type, self.data, self.priority)
            self.subtasks_list.append(subtask)

    @staticmethod
    def does_task_type_exists_in_system(task_type):
        for _, supported_task_types_list in ROBOT_USECASE_MAP.items():
            if(task_type in supported_task_types_list):
                return True

        return False

    @staticmethod
    def is_task_type_scenario(task_type):
        return task_type in SCENARIO_LIST