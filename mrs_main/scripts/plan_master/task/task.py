from constants.robots_usecases import ROBOT_USECASE_MAP
from constants.scenario_list import SCENARIO_LIST

from mrs_msgs.msg import TaskDesc

import uuid


class Task():
    """ Task class - representation of Task_desc message.
    Priority:
        - int from 0 to 10
        - 0 - most important
        -  10 - least important
    """
    def __init__(self, task_type, data, tags = [] ,priority=10, goal_handle=None):
        self.type = task_type
        self.priority = priority
        self.id = str(uuid.uuid4())  # uniqe id generator
        self.tags = tags
        self.data = data
        self.fulfilled = False
        self.goal_handle = goal_handle
        self.is_scenario = self._determine_if_task_is_scenario()
        self._add_distinguisher_to_id()
        self.assigned_robot_name = None
        if self.is_scenario:
            self._generate_subtasks()

    def parse_task_from_msg(self, msg):
        self.type = msg.type
        # self.data = None
        self.priority = msg.prioirty

    def return_task_desc_msg(self):
        # Debug!
        # self.data = None
        task_msg = TaskDesc()
        task_msg.data = ["x: "+str(self.data.pose.position.x) + "y: " \
            + str(self.data.pose.position.y)]
        task_msg.priority = self.priority
        task_msg.type = self.type
        return task_msg

    def is_suitable_for_robot(self, robot):
        return self.type in ROBOT_USECASE_MAP[robot.robot_type]

    def has_higher_priority_than(self, other_task):
        return self.priority < other_task.priority

    def _determine_if_task_is_scenario(self):
        return self.type in SCENARIO_LIST

    def _generate_subtasks(self):
        from plan_master.task.subtask import Subtask
        self.subtasks_list = []
        for subtask_desc in SCENARIO_LIST[self.type]["subtasks"]:
            # TODO remeber tag_name also for scenarios!
            subtask = Subtask(
                subtask_desc,
                self.data[subtask_desc["appropriate data index"]],
                parent_id=self.id,
                priority=self.priority,
                tags=[self.tags[subtask_desc["appropriate data index"]]]
                )
            self.subtasks_list.append(subtask)

    def _add_distinguisher_to_id(self):
        if self.is_scenario:
            self.id += "-scenario"
        else:
            self.id += "-ordinary"
    @staticmethod
    def does_task_type_exists_in_system(task_type):
        for _, supported_task_types_list in ROBOT_USECASE_MAP.items():
            if (task_type in supported_task_types_list):
                return True

        return False

    @staticmethod
    def is_task_type_scenario(task_type):
        return task_type in SCENARIO_LIST
