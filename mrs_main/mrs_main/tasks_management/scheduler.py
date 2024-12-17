from typing import List

from mrs_main.tasks_management.dependency_manager import DependencyManager
from mrs_main.tasks_management.task_fsm import TaskFSM

class Scheduler:
    def __init__(self, dependency_manager: DependencyManager):
        """ Manages the order and timing of task execution
        """

        # TODO: scheduler should decide if task can be executed or not based on the dependencies
        #           and current task status
        #      Additionally scheduler should allow only one task to be executed at the same time,
        #      other tasks should be planned or supervised (while other agents execute it)
        self._dependency_manager = dependency_manager
        self.backlog: List[TaskFSM] = [] #queue of tasks scheduled for execution - possibly should be thread safe

    def append_task(self, task_fsm: TaskFSM):
        """ Appends a new task to the task queue """
        self.backlog.append(task_fsm)

    def handle_current_task_finished(self, task_id: int):
        """ Handles the task finished event """
        if not any(task_fsm.task_data.short_id == task_id for task_fsm in self.backlog):
            return 
        assert self.backlog[0] is not None
        # assert self.backlog[0].task_data.short_id == task_id # TODO: this should be true everytime 
                                                                    # for now backlog is not managed
        self.backlog.pop(0) # task finished - remove from scheduler backlog

    def get_next_task(self):
        """ Returns the next task to be executed """
        #TODO: implement the logic to pull the most appropriate task from the backlog
        # dummy implementation - get next one in FIFO manner
        for task in self.backlog:
            if self._dependency_manager.are_task_dependencies_met(task.task_data.short_id):
                task.resume_after_finished_dependencies()
                break # TODO: need to handle the case when no task can be executed
