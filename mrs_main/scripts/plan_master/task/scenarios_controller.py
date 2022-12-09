from plan_master.task.task import Task
from plan_master.task.subtask  import Subtask
from mrs_msgs.msg import TaskStatus

class ScenariosController:
    def __init__(self, robots_harmonizers_list):
        """
        Responsible for handling changes in scenarios
        """
        self.subtask_harmonizer_dict = {}

    def check_if_conditions_are_met(self, task_id):
        pass

    def handle_completed_subtask(self, task_status: TaskStatus):
        req_task_id = task_status.id.id
        req_subtask_index = task_status.id.index    # subtask index within scenario
        for subtask, robot in self.subtask_harmonizer_dict.items():
            # looking for subtasks of specific scenario
            if subtask.id == req_task_id:
                req_found= subtask.mark_req_met(req_subtask_index)
                if(req_found):
                    # let know the robot that state of 
                    # requirement has changed to fulfilled  
                    self._send_scenario_signal(robot)

    def _send_scenario_signal(self, robot):
        print("Task ended - sending signal to robots!")
        robot.receive_scenario_signal()

    @staticmethod
    def get_tasks_groups_for_one_robot(scenario):
        """ 
        Method returns dict with:
            - key: index of first task to be executed 
            - value: group(list) of subtasks to be executed
                by the same robot 
                
        Abstract: some subtasks of scenario requires to be 
        executed by the same robot
        """
        tasks_group_for_same_robot = {}
        for subtask in scenario.subtasks_list:
            if subtask.same_robot_as_task_req is None:
                # create new group of tasks 
                tasks_group_for_same_robot[subtask.index] = [subtask]
            
            else:
                other_task_index = subtask.same_robot_as_task_req
                tasks_group_for_same_robot[other_task_index].append(subtask)

        return tasks_group_for_same_robot
