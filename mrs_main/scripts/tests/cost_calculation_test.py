# i rt(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from plan_master.robot_task_harmonizer import RobotTaskHarmonizer
from plan_master.task.task import Task
from tests.fake_test_robot import FakeRobot
from tests.constats import *
from constants import *
import pytest 
from enum import Enum

@pytest.fixture
def base_robot_task_harmoniser():
    fake_robot = FakeRobot()
    robot_task_harmoniser = RobotTaskHarmonizer(fake_robot.robot_name)
    #incjecting fake robot
    robot_task_harmoniser.robot = fake_robot
    return robot_task_harmoniser

def test_cost_estimation_and_task_placement_only_one_new_task(base_robot_task_harmoniser):
    fake_task = Task("GT",None)
    fake_cost, position  = base_robot_task_harmoniser.get_estimated_task_cost_with_scheduled_position(fake_task)
    assert(fake_cost == FAKE_BIG_LENGTH)
    assert(position == 0)


def test_cost_estimation_and_task_placement_new_task_one_in_backlog(base_robot_task_harmoniser):
    fake_task = Task("GT",None)
    base_robot_task_harmoniser.task_list.append(fake_task)
    fake_cost, position = base_robot_task_harmoniser.get_estimated_task_cost_with_scheduled_position(fake_task)
    assert(fake_cost == FAKE_BIG_LENGTH+FAKE_SMALL_LENGTH)
    assert(position == -1)

def test_cost_estimation_and_task_placement_with_task_reorder(base_robot_task_harmoniser):
    fake_task_high_priority = Task("GT",None, 8)
    fake_task_middle_priority = Task("GT",None, 5)
    fake_task_low_priority = Task("GT", None, 3)
    
    base_robot_task_harmoniser.task_list.append(fake_task_high_priority)
    base_robot_task_harmoniser.task_list.append(fake_task_low_priority)
    base_robot_task_harmoniser.task_list.append(fake_task_low_priority)
    fake_cost, position = base_robot_task_harmoniser.get_estimated_task_cost_with_scheduled_position(fake_task_middle_priority)
    assert(fake_cost == FAKE_BIG_LENGTH+FAKE_SMALL_LENGTH+FAKE_SMALL_LENGTH/DELAY_TASK_PENALTY
        +FAKE_SMALL_LENGTH/DELAY_TASK_PENALTY)
    assert(position == 1 )

