from typing import List
from threading import RLock

from mrs_main.common.synchronization import synchronized
from mrs_main.tasks_management.dependency_manager import DependencyManager
from mrs_main.tasks_management.task_fsm import TaskFSM

class Scheduler:
    def __init__(self, dependency_manager: DependencyManager):
        """ Manages the order and timing of task execution
        """

        #      Additionally scheduler should allow only one task to be executed at the same time,
        #      other tasks should be planned or supervised (while other agents execute it)
        self._dependency_manager = dependency_manager
        self.current_task = None
        self.backlog: List[TaskFSM] = [] # queue of tasks scheduled for execution
        self._backlog_lock = RLock()

    @synchronized(lock_attr_name='_backlog_lock')
    def append_task(self, task_fsm: TaskFSM):
        """ Appends a new task to the task queue """
        self.backlog.append(task_fsm)
        if self.current_task is None:
            self.get_next_task()

    @synchronized(lock_attr_name='_backlog_lock')
    def handle_current_task_finished(self, task_id: int):
        """ Handles the task finished event """
        if not any(task_fsm.task_data.short_id == task_id for task_fsm in self.backlog):
            return 
        assert self.backlog[0] is not None
        assert self.backlog[0].task_data.short_id == task_id # make sure the proper task is at the top of the backlog

        self.backlog.pop(0) # task finished - remove from scheduler backlog
        self.get_next_task()
    
    @synchronized(lock_attr_name='_backlog_lock')
    def get_next_task(self):
        """ Returns the next task to be executed """
        #TODO: implement the logic to pull the most appropriate task from the backlog
        # dummy implementation - get next one in FIFO manner
        for idx, task in enumerate(self.backlog):
            if self._dependency_manager.are_task_dependencies_met(task.task_data.short_id):
                task.resume_after_finished_dependencies()
                # Move the task to the first position in the backlog
                self.backlog.insert(0, self.backlog.pop(idx))
                return

        self.current_task = None
