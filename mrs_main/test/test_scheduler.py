import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.scheduler import Scheduler
from mrs_main.tasks_management.task_fsm import TaskFSM

@pytest.fixture
def scheduler():
    dependency_manager = MagicMock()
    dependency_manager.are_task_dependencies_met = MagicMock(return_value=True)
    return Scheduler(dependency_manager)

def create_mock_task_fsm(short_id):
    task_fsm = MagicMock(spec=TaskFSM)
    task_fsm.task_data = MagicMock()
    task_fsm.task_data.short_id = short_id
    task_fsm.resume_after_finished_dependencies = MagicMock()
    return task_fsm

def test_append_task(scheduler):
    task = create_mock_task_fsm(1)
    scheduler.append_task(task)
    assert task in scheduler.backlog

def test_get_next_task(scheduler):
    task1 = create_mock_task_fsm(1)
    task2 = create_mock_task_fsm(2)

    scheduler.append_task(task1)
    scheduler.append_task(task2)

    scheduler.get_next_task()

    assert task1.resume_after_finished_dependencies.called # as it gets removed from the queue
    assert not task2.resume_after_finished_dependencies.called

def test_handle_current_task_finished(scheduler):
    task = create_mock_task_fsm(1)
    scheduler.append_task(task)
    
    scheduler.handle_current_task_finished(task_id=task.task_data.short_id)
    
    assert task not in scheduler.backlog
