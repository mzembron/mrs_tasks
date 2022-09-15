from plan_master.turtlebot import Turtlebot


###  
# Wrapper for robot class with purpose of managing tasks
###

class RobotTaskHarmoniser:
    def __init__(self,robot_name):
        ### just assume we use only turtlebots:
        self.robot = Turtlebot(robot_name)
        self.task_dict = {}

    def get_estimated_task_end_time(self, goal):
        if self.robot.current_goal is not None:
            #cost from task end position to this goal
            return (self.robot.get_operation_cost_to_current_goal()
                + self.robot.get_operation_cost_from_current_goal_to_spec_position(goal))
    def add_task(self):
        pass