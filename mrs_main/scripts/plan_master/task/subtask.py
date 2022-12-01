from plan_master.task.task import Task


class Subtask(Task):
    """Subtask - main purpose of this task is to create ."""

    def __init__(self, subtask_param_dict, data, priority=10):
        """Subtask."""
        super().__init__(subtask_param_dict['type'], data, priority)
        self.index = \
            subtask_param_dict['index']
        self.tasks_indicies_required_to_start = \
            subtask_param_dict['requires to start']
        self.tasks_indicies_required_to_stop = \
            subtask_param_dict['requires to end']
        self.index_of_task_with_same_robot_requirement = \
            subtask_param_dict['same robot as task']

    # def assign_id(self, id):
    #     self.subtask_id
