# i rt(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from plan_master.robot_task_harmoniser import RobotTaskHarmoniser
from plan_master.task.task import Task
from tests.fake_test_robot import FakeRobot
from tests.constats import FAKE_BIG_LENGTH, FAKE_SMALL_LENGTH
import pytest 


def test_cost_estimation_only_one_new_task():
    fake_robot = FakeRobot()
    robot_task_harmoniser = RobotTaskHarmoniser(fake_robot.robot_name)
    #incjecting fake robot
    robot_task_harmoniser.robot = fake_robot
    fake_task = Task("GT",None)
    fake_cost = robot_task_harmoniser.get_estimated_task_cost(fake_task)
    assert(fake_cost == FAKE_BIG_LENGTH)


def test_cost_estimation_new_task_one_in_backlog():
    fake_robot = FakeRobot()
    robot_task_harmoniser = RobotTaskHarmoniser(fake_robot.robot_name)
    #incjecting fake robot
    robot_task_harmoniser.robot = fake_robot
    fake_task = Task("GT",None)
    robot_task_harmoniser.task_list.append(fake_task)
    fake_cost = robot_task_harmoniser.get_estimated_task_cost(fake_task)
    assert(fake_cost == FAKE_BIG_LENGTH+FAKE_SMALL_LENGTH)