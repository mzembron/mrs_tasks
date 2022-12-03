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
        print("sending signal to robot!")
        robot.receive_scenario_signal()