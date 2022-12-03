import pytest 
from plan_master.task.task import Task

GO_TO_TYPE = 'GT'
FAKE_TASK_TYPE = 'FT'
TEST_DATA = ['kitchen']
ORDINARY_TASK_DISTINGUISHER = 'ordinary'

def test_spot_fake_task_type():
    assert(not Task.does_task_type_exists_in_system(FAKE_TASK_TYPE))

def test_find_real_task_type():
    assert(Task.does_task_type_exists_in_system(GO_TO_TYPE))

def test_id_distingusher():
    test_task = Task(GO_TO_TYPE, TEST_DATA)
    assert(test_task.id.endswith(ORDINARY_TASK_DISTINGUISHER))
    