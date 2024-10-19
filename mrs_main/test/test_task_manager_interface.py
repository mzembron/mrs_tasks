import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.task_manager_interface import TaskManagerInterface

class TestTaskManagerInterface:

    @pytest.fixture
    def setup(self):
        self.task_dict = {}
        self.concrete_task_manager = MagicMock()
        self.concrete_task_manager._task_dict = self.task_dict
        self.concrete_task_manager._dependency_manager = MagicMock()
        self.concrete_task_manager.intrest_desc = MagicMock(spec=IntrestDescription)
        self.task_manager_interface = TaskManagerInterface(self.concrete_task_manager)

    def test_init(self, setup):
        assert self.task_manager_interface._task_dict == self.task_dict
        assert self.task_manager_interface._TaskManagerInterface__concrete_task_manager == self.concrete_task_manager

    def test_task_dict_property(self, setup):
        assert self.task_manager_interface.task_dict == self.task_dict

    def test_receive_task(self, setup):
        task_desc = '{"desc": "Test Task", "dependencies": []}'
        task_finished_callback = MagicMock()
        self.task_manager_interface.receive_task('task_1', task_desc, task_finished_callback)
        
        assert 'task_1' in self.task_dict
        assert self.task_dict['task_1']._task_desc == task_desc

    def test_get_intrest(self, setup):
        self.concrete_task_manager.intrest_desc = MagicMock(spec=IntrestDescription)
        
        result = self.task_manager_interface.get_intrest(1)
        
        assert result == self.concrete_task_manager.intrest_desc

    def test_define_next_behavior(self, setup):
        # TODO: refactor this test
        task_conv_msg = MagicMock(spec=TaskConvMsg)
        task_conv_msg.short_id = 1
        
        task = MagicMock(spec=TaskFSM)
        task.get_next_message.return_value = 'response'
        
        self.task_dict[task_conv_msg.short_id] = task
        
        result = self.task_manager_interface.define_next_behavior(task_conv_msg)
        
        assert result == 'response'
        task.get_next_message.assert_called_once_with(msg=task_conv_msg)