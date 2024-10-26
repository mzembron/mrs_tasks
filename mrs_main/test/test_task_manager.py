import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.task_manager import TaskManager

import json

class TestTaskManager:

    @pytest.fixture
    def setup(self):
        self.task_manager = TaskManager('test_agent')
        self.task_manager._dependency_manager = MagicMock()
        self.task_manager.intrest_desc = MagicMock(spec=IntrestDescription)
        self.task_dict = {}
        self.task_manager._task_dict = self.task_dict

    def test_receive_task(self, setup):
        task_desc = '{"desc": "Test Task", "dependencies": []}'
        task_finished_callback = MagicMock()
        self.task_manager.receive_task('task_1', task_desc, task_finished_callback)
        
        assert 'task_1' in self.task_dict
        assert self.task_dict['task_1']._task_data.task_desc == json.loads(task_desc)

    def test_get_intrest(self, setup):
        self.task_manager.intrest_desc = MagicMock(spec=IntrestDescription)
        task_desc = '{"desc": "Test Task", "dependencies": []}'
        task_finished_callback = MagicMock()
        self.task_manager.receive_task('task_1', task_desc, task_finished_callback)
        result = self.task_manager.get_intrest(1)
        
        # TODO: this test should be corrected after replacing the fake intrest desc
        # with proper implementation 
        assert result == self.task_manager.intrest_desc

    def test_define_next_behavior(self, setup):
        # TODO: refactor this test
        task_conv_msg = MagicMock(spec=TaskConvMsg)
        task_conv_msg.short_id = 1
        
        task = MagicMock(spec=TaskFSM)
        task.get_next_message.return_value = 'response'
        
        self.task_dict[task_conv_msg.short_id] = task
        
        result = self.task_manager.define_next_behavior(task_conv_msg)
        
        assert result == 'response'
        task.get_next_message.assert_called_once_with(msg=task_conv_msg)