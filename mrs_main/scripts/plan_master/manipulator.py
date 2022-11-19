import math

import rospy
from nav_msgs.msg import  Odometry
# from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult


from helpful_functions.data_manipulation import DataManipulation
from constants.robot_type import RobotType
from constants.constants import FAKE_MANIPULATOR_ANGULAR_VELOCITY, ORIENTATION_ACCURACY
import math

class Manipulator:

    def __init__(self, robot_name):
        rospy.Subscriber(robot_name + "/odom", Odometry, self._odom_callback)
        self.velocity_publisher = rospy.Publisher(robot_name + "/cmd_vel", Twist, queue_size=10)
        self.outcome_publisher = rospy.Publisher(robot_name+"/move_base/result", MoveBaseActionResult, queue_size=10)
        self.robot_name = robot_name
        self.robot_type = RobotType.MANIPULATOR
        self.current_goal = None
    
    def calc_cost_from_curr_position_to_spec_position(self, goal_odom_data):
        #this is important
        yaw_delta = self._calculate_yaw_difference(self.current_odom_data, goal_odom_data)
        return yaw_delta/FAKE_MANIPULATOR_ANGULAR_VELOCITY

    def calc_cost_from_curr_position_to_curr_goal(self):
        assert(self.current_goal is not None)

        # plan = self._make_path_plan(self.current_odom_data.pose, self.current_goal)
        # return self._approximate_path_length(plan)

    def calc_cost_from_spec_position_to_spec_position(self, start_odom_data, goal_odom_data):
        #this is important
        yaw_delta = self._calculate_yaw_difference(start_odom_data, goal_odom_data)
        return yaw_delta/FAKE_MANIPULATOR_ANGULAR_VELOCITY

    def calc_cost_from_curr_goal_to_spec_position(self, goal_odom_data):
        assert(self.current_goal is not None)

        # plan = self._make_path_plan(self.current_goal, goal_odom_data)
        # return self._approximate_path_length(plan)

    def go_to_goal(self, goal_odom_data):
        yaw_delta = self._calculate_yaw_difference(self.current_odom_data, goal_odom_data)
        sign = yaw_delta/abs(yaw_delta)
        self.current_goal = goal_odom_data
        self._start_manipulation(sign)

    def _start_manipulation(self, rotation_direction = 1):
        msg = Twist()
        msg.angular.z = rotation_direction * FAKE_MANIPULATOR_ANGULAR_VELOCITY
        self.velocity_publisher.publish(msg)

    def _stop_manipulation(self):
        msg = Twist()
        msg.angular.z = 0
        self.velocity_publisher.publish(msg)

    def _odom_callback(self, data):
        self.current_odom_data = data.pose
        if(self.current_goal is not None):
            yaw_delta = self._calculate_yaw_difference(self.current_odom_data, self.current_goal)
            if (abs(yaw_delta) < ORIENTATION_ACCURACY):
                self._stop_manipulation()
                self._publish_action_outcome()
                self.current_goal = None

    def _publish_action_outcome(self):
        msg = MoveBaseActionResult()
        msg.status.status = 3
        self.outcome_publisher.publish(msg)

    def _calculate_yaw_difference(self, start_odom_data, goal_odom_data):
        goal_orientation_q = goal_odom_data.pose.orientation
        current_orientation_q = start_odom_data.pose.orientation 

        goal_orientation_list = [goal_orientation_q.x, goal_orientation_q.y, goal_orientation_q.z, goal_orientation_q.w]
        current_orientation_list = [current_orientation_q.x, current_orientation_q.y, current_orientation_q.z, current_orientation_q.w]

        (_, _, yaw_goal) = euler_from_quaternion(goal_orientation_list)
        (_, _, yaw_current) = euler_from_quaternion(current_orientation_list)

        # take care about modulo, be aware of changing sign
        return yaw_goal - yaw_current








