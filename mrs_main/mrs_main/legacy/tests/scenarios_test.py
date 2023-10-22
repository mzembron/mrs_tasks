import pytest 
from plan_master.task.task import Task

from constants.scenario_list import SCENARIO_LIST

BRING_GOOD_TYPE = 'BG'
FAKE_SCENARIO_TYPE = 'FS'
GO_TO_TYPE = 'GT'
MANIPULATE_TYPE = 'MT'
TEST_DATA = ["botle", "Tomek"]
SCENARIO_DISTINGUISHER = 'scenario'

def test_subtask_generation():
    bring_good_scenario = Task(BRING_GOOD_TYPE, TEST_DATA)
    assert(bring_good_scenario.is_scenario)
    subtasks_types = []
    subtask_list = bring_good_scenario.subtasks_list
    for subtask in subtask_list:
        subtasks_types.append(subtask.type)
    assert(bring_good_scenario.id.endswith(SCENARIO_DISTINGUISHER))
    assert(GO_TO_TYPE in subtasks_types)
    assert(MANIPULATE_TYPE in subtasks_types)
    for task_index in range(len(subtask_list)):
        task = subtask_list[task_index]
        assert(task.type == SCENARIO_LIST[BRING_GOOD_TYPE]["subtasks"][task_index]["type"])
        assert(task.data == TEST_DATA[SCENARIO_LIST[BRING_GOOD_TYPE]["subtasks"][task_index]["appropriate data index"]])

        assert(bring_good_scenario.id == task.id)
        assert(task_index == task.index)

def test_spot_real_scenario():
    assert(Task.is_task_type_scenario(BRING_GOOD_TYPE))

def test_spot_fake_scenario():
    assert(not Task.is_task_type_scenario(FAKE_SCENARIO_TYPE))
