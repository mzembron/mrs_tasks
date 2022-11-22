from constants.testing_constats import FAKE_BIG_LENGTH, FAKE_SMALL_LENGTH


class FakeRobot:

    def __init__(self):
        self.robot_name = "fake_robot"

    def calc_cost_from_curr_position_to_spec_position(self, goal_odom_data):
        return FAKE_BIG_LENGTH

    def calc_cost_from_spec_position_to_spec_position(self, start_odom_data, goal_odom_data):
        return FAKE_SMALL_LENGTH
