import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.scheduler import Scheduler
from mrs_main.tasks_management.task import Task

@pytest.fixture
def scheduler():
    dependency_manager = MagicMock()
    dependency_manager.are_task_dependencies_met = MagicMock(return_value=True)
    return Scheduler(dependency_manager)

def create_mock_task(short_id):
    task = MagicMock(spec=Task)
    task.short_id = short_id
    task.fsm = MagicMock()
    task.fsm.resume_after_finished_dependencies = MagicMock()
    return task

def test_append_task(scheduler):
    task = MagicMock(spec=Task)
    task.short_id = "task1"
    scheduler.append_task(task)
    assert task in scheduler.backlog

def test_get_next_task(scheduler):
    task1 = create_mock_task("task1")
    task2 = create_mock_task("task2")

    scheduler.append_task(task1)
    scheduler.append_task(task2)

    scheduler._dependency_manager.are_task_dependencies_met = MagicMock(return_value=True)

    scheduler.get_next_task()

    assert task1.fsm.resume_after_finished_dependencies.called # as it gets removed from the queue
    assert not task2.fsm.resume_after_finished_dependencies.called

def test_handle_current_task_finished(scheduler):
    task = MagicMock(spec=Task)
    task.short_id = "task1"
    scheduler.append_task(task)
    
    scheduler.handle_current_task_finished(task)
    
    assert task not in scheduler.backlog
