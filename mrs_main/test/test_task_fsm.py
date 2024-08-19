import pytest
from unittest.mock import MagicMock
from mrs_main.tasks_management.task_fsm import TaskFSM, State, DefineTaskIntrest
from mrs_main.common.objects import TaskConvMsg
from mrs_main.common.conversation_data import MrsConvPerform
from mrs_main.common.exceptions import InvalidMsgPerformative

def test_initialization():
    fsm = TaskFSM()
    assert isinstance(fsm._state, DefineTaskIntrest)
    assert fsm._state.task == fsm

def test_transition_to():
    fsm = TaskFSM()
    new_state = MagicMock(spec=State)
    fsm.transition_to(new_state)
    assert fsm._state == new_state
    assert new_state.task == fsm
    new_state.change_state_routine.assert_called_once()

class TestState:

    @pytest.fixture(autouse=True)
    def setup(self):
        self.state = State()
        self.state.task = MagicMock(spec=TaskFSM)

    def test_define_next(self):
        msg = TaskConvMsg()
        msg.performative = MrsConvPerform.declare_coord_intrest
        self.state.respond_to_coord_intrest_declaration = MagicMock(return_value="response")
        response = self.state.define_next(msg)
        assert response == 'response'
        self.state.respond_to_coord_intrest_declaration.assert_called_once_with(msg)

    def test_invalid_performative(self):
        msg = TaskConvMsg()
        msg.performative='invalid_performative'
        with pytest.raises(InvalidMsgPerformative):
            self.state.define_next(msg)

if __name__ == '__main__':
    pytest.main()