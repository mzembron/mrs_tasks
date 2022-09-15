from plan_master.turtlebot import Turtlebot


###  
# Wrapper for robot class with purpose of managing tasks
###

class RobotTaskHarmoniser:
    def __init__(self,robot_name):
        ### just assume we use only turtlebots:
        self.robot = Turtlebot(robot_name)
        self.task_dict = {}

    def get_estimated_task_end_time(self):
        pass

    def add_task(self):
        pass