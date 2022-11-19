import pytest 
from plan_master.task.task import Task


BRING_GOOD_TYPE = 'BG'
FAKE_SCENARIO_TYPE = 'FS'
GO_TO_TYPE = 'GT'
MANIPULATE_TYPE = 'MT'


def test_subtask_generation():
    fake_scenario = Task(BRING_GOOD_TYPE, "kitchen")
    assert(fake_scenario.is_scenario)
    subtasks_types = []
    for subtask in fake_scenario.subtasks_list:
        subtasks_types.append(subtask.type)

    assert(GO_TO_TYPE in subtasks_types)
    assert(MANIPULATE_TYPE in subtasks_types)

def test_spot_real_scenario():
    assert(Task.is_task_type_scenario(BRING_GOOD_TYPE))

def test_spot_fake_scenario():
    assert(not Task.is_task_type_scenario(FAKE_SCENARIO_TYPE))