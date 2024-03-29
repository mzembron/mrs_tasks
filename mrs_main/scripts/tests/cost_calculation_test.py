# i rt(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from plan_master.task_harmonizer import TaskHarmonizer
from plan_master.task.task import Task
from tests.fake_test_robot import FakeRobot
from constants.testing_constats import *
from constants.constants import *
import pytest
from enum import Enum

@pytest.fixture
def base_robot_task_harmoniser():
    fake_robot = FakeRobot()
    robot_task_harmoniser = TaskHarmonizer(fake_robot)
    #incjecting fake robot
    robot_task_harmoniser.robot = fake_robot
    return robot_task_harmoniser

""" Simple task tests: """

def test_cost_estimation_and_task_placement_only_one_new_task_sc(base_robot_task_harmoniser):
    fake_task = Task("GT",None)
    execution_estimation_list  = base_robot_task_harmoniser.get_execution_estimation_of(fake_task)
    assert(len(execution_estimation_list) == 1)
    task_execution_estimation = execution_estimation_list[0]
    assert(task_execution_estimation.full_cost == FAKE_BIG_LENGTH)
    assert(task_execution_estimation.task_position == 0)


def test_cost_estimation_and_task_placement_new_task_one_in_backlog_sc(base_robot_task_harmoniser):
    fake_task = Task("GT",None)
    base_robot_task_harmoniser.task_list.append(fake_task)
    execution_estimation_list  = base_robot_task_harmoniser.get_execution_estimation_of(fake_task)
    assert(len(execution_estimation_list) == 1)
    task_execution_estimation = execution_estimation_list[0]
    assert(task_execution_estimation.full_cost == FAKE_BIG_LENGTH+FAKE_SMALL_LENGTH)
    assert(task_execution_estimation.task_position == 1)

def test_cost_estimation_and_task_placement_with_task_reorder_sc(base_robot_task_harmoniser):
    fake_task_high_priority = Task("GT",None, 3)
    fake_task_middle_priority = Task("GT",None, 5)
    fake_task_low_priority = Task("GT", None, 8)
    
    base_robot_task_harmoniser.task_list.append(fake_task_high_priority)
    base_robot_task_harmoniser.task_list.append(fake_task_low_priority)
    base_robot_task_harmoniser.task_list.append(fake_task_low_priority)
    execution_estimation_list  = base_robot_task_harmoniser.get_execution_estimation_of(fake_task_middle_priority)
    assert(len(execution_estimation_list) == 1)
    task_execution_estimation = execution_estimation_list[0]
    assert(task_execution_estimation.full_cost == FAKE_BIG_LENGTH+FAKE_SMALL_LENGTH+FAKE_SMALL_LENGTH/DELAY_TASK_PENALTY
        +FAKE_SMALL_LENGTH/DELAY_TASK_PENALTY)
    assert(task_execution_estimation.task_position == 1)

""" Scenario tests: """

