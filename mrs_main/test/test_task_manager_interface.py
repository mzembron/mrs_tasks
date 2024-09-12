import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.task import Task
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.task_manager_interface import TaskManagerInterface

class TestTaskManagerInterface:

    @pytest.fixture
    def setup(self):
        self.task_dict = {}
        self.concrete_task_manager = MagicMock()
        self.task_manager_interface = TaskManagerInterface(self.task_dict, self.concrete_task_manager)

    def test_init(self, setup):
        assert self.task_manager_interface._task_dict == self.task_dict
        assert self.task_manager_interface._TaskManagerInterface__concrete_task_manager == self.concrete_task_manager

    def test_task_dict_property(self, setup):
        assert self.task_manager_interface.task_dict == self.task_dict

    def test_receive_task(self, setup):
        task = MagicMock(spec=Task)
        task.short_id = 'task_1'
        task.desc = 'Test Task'
        
        self.task_manager_interface.receive_task(task)
        
        assert self.task_dict['task_1'] == task

    def test_get_intrest(self, setup):
        self.concrete_task_manager.intrest_desc = MagicMock(spec=IntrestDescription)
        
        result = self.task_manager_interface.get_intrest(1)
        
        assert result == self.concrete_task_manager.intrest_desc

    def test_define_next_behavior(self, setup):
        task_conv_msg = MagicMock(spec=TaskConvMsg)
        task_conv_msg.short_id = 'task_1'
        
        task = MagicMock(spec=Task)
        task.get_response.return_value = 'response'
        
        self.task_dict['task_1'] = task
        
        result = self.task_manager_interface.define_next_behavior(task_conv_msg)
        
        assert result == 'response'
        task.get_response.assert_called_once_with(task_conv_msg)