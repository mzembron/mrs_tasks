from plan_master.task.task import Task


class Subtask(Task):
    """Subtask - task that is part of a more complex task
        (called Scenario)"""

    def __init__(self, subtask_param_dict, data, parent_id, priority=10, tags = []):
        """Subtask.
            - id - subtask has same id as parent, subtasks can be
                distinguished by theis indices
            - index - allows easy identification and reference
                within a specific scenario
            - required_to_start_if_met_dict - dict containing
                indices of other subtask that are required before
                this one can be started
            - required_to_stop_if_met_dict - same as for the above
                but refers to end conditions
            - same_robot_as_task_req -index of task which requires
                same robot to be used """
        super().__init__(subtask_param_dict['type'], data, priority=priority, tags=tags)
        self.id = parent_id
        self.index = \
            subtask_param_dict['index']
        self.required_to_start_if_met_dict = \
            self._parse_task_requirements(
                subtask_param_dict['requires to start']
            )
        self.required_to_end_if_met_dict = \
            self._parse_task_requirements(
                subtask_param_dict['requires to end']
            )
        self.same_robot_as_task_req = \
            subtask_param_dict['same robot as task']

    def mark_req_met(self, task_req_index) -> bool:
        """ changes requirements state to fullfiled
             - returns True if state of requirement has changed
             - returns False otherwise """

        for task_index, _ in self.required_to_start_if_met_dict.items():
            if task_index == task_req_index:
                print('Requirement of task ', task_req_index,\
                ' ending for task ', self.index, ' fullfilled!')
                self.required_to_start_if_met_dict[task_index] = True
                return True# can be found only once (cannot be in start and end req)

        for task_index, _ in self.required_to_end_if_met_dict.items():
            if task_index == task_req_index:
                self.required_to_end_if_met_dict[task_index] = True
                print('Requirement of task ', task_req_index,
                ' ending for task ', self.index, ' fullfilled!')
                return True# can be found only once



        return False

    def is_start_req_met(self):
        for _, is_met in self.required_to_start_if_met_dict.items():
            if (not is_met):
                return False
        return True

    def is_end_req_met(self):
        for _, is_met in self.required_to_end_if_met_dict.items():
            if (not is_met):
                return False
        return True
    
    def _parse_task_requirements(self, req_list):
        req_if_met_dict = {}
        for requirement in req_list:
            req_if_met_dict[requirement] = False

        return req_if_met_dict
