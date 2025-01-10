import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.task_fsm import TaskFSM
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.tasks_management.task_manager import TaskManager
from mrs_main.knowledge_base.knowledge_base import KnowledgeBase

import json

class TestTaskManager:

    @pytest.fixture
    def setup(self):
        self.test_agent_type = 0
        self.task_manager = TaskManager('test_agent', agent_type=self.test_agent_type)
        self.task_manager._dependency_manager = MagicMock()
        self.task_manager.intrest_desc = MagicMock(spec=IntrestDescription)
        self.task_dict = {}
        self.task_manager._task_dict = self.task_dict

    def test_receive_task(self, setup):
        task_desc = '{"desc": "Test Task", "dependencies": []}'
        task_finished_callback = MagicMock()
        self.task_manager.receive_task(1, task_desc, task_finished_callback)
        
        assert 1 in self.task_dict
        assert self.task_dict[1].task_data.task_desc == json.loads(task_desc)

    def test_get_intrest(self, setup):
        task_desc = '{"desc": "Test Task", "dependencies": []}'
        test_task_id = 0
        task_finished_callback = MagicMock()
        self.task_manager.receive_task(test_task_id, task_desc, task_finished_callback)
        result = self.task_manager.get_intrest(test_task_id)
        
        # TODO: this test should be corrected after replacing the fake intrest desc
        # with proper implementation 
        expected_result = KnowledgeBase(self.test_agent_type).get_intrest_desc(self.task_manager._task_dict[test_task_id].task_data)
        assert result.coordination == expected_result.coordination
        assert result.execution == expected_result.execution

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