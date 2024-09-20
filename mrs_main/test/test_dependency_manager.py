import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.dependency_manager import DependencyManager, TaskDependencyManager
from mrs_main.tasks_management.task import Task

class TestDependencyManager:

    @pytest.fixture
    def setup(self):
        tasks_dict = {}
        tasks_dict[1] = MagicMock(spec=Task)
        tasks_dict[2] = MagicMock(spec=Task)
        self.dependency_manager = DependencyManager(tasks_dict)

    def test_are_task_dependencies_met(self, setup):
        self.dependency_manager.introduce_task_dependencies(1, [])
        assert self.dependency_manager.are_task_dependencies_met(1) == True

        self.dependency_manager.introduce_task_dependencies(2, [1])
        assert self.dependency_manager.are_task_dependencies_met(2) == False

    def test_update_dependencies(self, setup):
        self.dependency_manager.introduce_task_dependencies(1, [])
        self.dependency_manager.introduce_task_dependencies(2, [1])

        self.dependency_manager.update_dependencies(1)
        assert self.dependency_manager.are_task_dependencies_met(2) == True

    def test_introduce_task_dependencies(self, setup):
        self.dependency_manager.introduce_task_dependencies(2, [])
        self.dependency_manager.introduce_task_dependencies(3, [])
        self.dependency_manager.introduce_task_dependencies(1, [2, 3])
        assert self.dependency_manager._tasks_dependencies.has_node(1)
        assert self.dependency_manager._tasks_dependencies.has_edge(2, 1)
        assert self.dependency_manager._tasks_dependencies.has_edge(3, 1)

class TestTaskDependencyManager:

    @pytest.fixture
    def setup(self):
        tasks_dict = {}
        tasks_dict[1] = MagicMock(spec=Task)
        tasks_dict[2] = MagicMock(spec=Task)
        tasks_dict[3] = MagicMock(spec=Task)
        self.dependency_manager = DependencyManager(tasks_dict)
        self.dependency_manager.introduce_task_dependencies(2, [])
        self.dependency_manager.introduce_task_dependencies(3, [])
        self.task_dependency_manager = TaskDependencyManager(self.dependency_manager, 1, [2, 3])

    def test_are_dependencies_met(self, setup):
        assert self.task_dependency_manager.are_dependencies_met() == False
        self.dependency_manager.update_dependencies(2)
        self.dependency_manager.update_dependencies(3)
        assert self.task_dependency_manager.are_dependencies_met() == True

    def test_notify_on_finish(self, setup):
        self.task_dependency_manager.notify_on_finish()
        assert self.dependency_manager._tasks_dependencies.nodes[1]['finished'] == True