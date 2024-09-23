import pytest
from unittest.mock import MagicMock

from mrs_main.tasks_management.task_fsm import TaskFSM, State, DefineTaskIntrest, WaitForExec, TaskCompleted
from mrs_main.common.objects import IntrestDescription, TaskConvMsg
from mrs_main.common.conversation_data import MrsConvPerform
from mrs_main.common.exceptions import InvalidMsgPerformative
from mrs_main.tasks_management.dependency_manager import TaskDependencyManager
from mrs_main.task_execution.task_executor import TaskExecutor

class TestTaskFSM:
    @pytest.fixture(autouse=True)
    def setup(self):
        self.dependency_manager = MagicMock(spec=TaskDependencyManager)
        self.task_executor = MagicMock(spec=TaskExecutor)
        self.interest_desc = MagicMock(spec=IntrestDescription)
        self.fsm = TaskFSM(self.dependency_manager, self.task_executor, self.interest_desc)

    def test_initialization(self):
        assert isinstance(self.fsm._state, DefineTaskIntrest)
        assert self.fsm._state.task_fsm == self.fsm

    def test_transition_to(self):
        new_state = MagicMock(spec=State)
        self.fsm.transition_to(new_state)
        assert self.fsm._state == new_state
        assert new_state.task_fsm == self.fsm
        new_state.change_state_routine.assert_called_once()


class TestState:

    @pytest.fixture(autouse=True)
    def setup(self):
        self.state = State()
        self.state.task_fsm = MagicMock(spec=TaskFSM)

    def test_define_next(self):
        msg = TaskConvMsg()
        msg.performative = MrsConvPerform.declare_coord_intrest
        self.state.respond_to_coord_intrest_declaration = MagicMock(return_value="response")
        response = self.state.define_next(msg)
        assert response == 'response'
        self.state.respond_to_coord_intrest_declaration.assert_called_once_with(msg)

    def test_invalid_performative(self):
        msg = TaskConvMsg()
        msg.performative = 'invalid_performative'
        with pytest.raises(InvalidMsgPerformative):
            self.state.define_next(msg)

    def test_change_state_to_task_completed(self):
        dependency_manager = MagicMock(spec=TaskDependencyManager)
        task_executor = MagicMock(spec=TaskExecutor)
        interest_desc = MagicMock(spec=IntrestDescription)
        self.task_fsm = TaskFSM(dependency_manager, task_executor, interest_desc)
        self.wait_for_exec = WaitForExec()
        self.wait_for_exec.task_fsm = self.task_fsm
        self.task_fsm._state = self.wait_for_exec
        self.wait_for_exec.change_state_routine()

        # Assert that the state has been changed to TaskCompleted
        assert isinstance(self.task_fsm._state, TaskCompleted)

if __name__ == '__main__':
    pytest.main()