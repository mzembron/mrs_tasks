from six.moves import input

import sys
import threading
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from math import pi, tau, dist, fabs, cos
# from franka_gripper import move_ac
from moveit_commander.conversions import pose_to_list

# Plan master imports 

from constants.robot_type import RobotType
from franka_gripper.msg import MoveActionGoal
from move_base_msgs.msg import MoveBaseActionResult

from constants.testing_constats import FAKE_BIG_LENGTH, FAKE_SMALL_LENGTH

class PandaManip:
    def __init__(self, robot_name):
        self.outcome_publisher = rospy.Publisher(robot_name+"/move_base/result", MoveBaseActionResult, queue_size=10)
        self.gripper_publisher = rospy.Publisher("/franka_gripper/move/goal", MoveActionGoal, queue_size=10)
        self.robot_name = robot_name
        self.robot_type = RobotType.MANIPULATOR
        self.current_goal = None
        self.thread_handles = []


        ##### PANDA TUTORIAL
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot_commander = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot_commander.get_group_names()
        print("============ Available Planning Groups:", robot_commander.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot_commander.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot_commander
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def __del__(self):
        for thr in self.thread_handles:
            if(thr.isAlive()):
                thr.join()
                print("Killing thread!")


    def calc_cost_from_curr_position_to_spec_position(self, goal_odom_data):
        return FAKE_BIG_LENGTH

    def calc_cost_from_spec_position_to_spec_position(self, start_odom_data, goal_odom_data):
        return FAKE_SMALL_LENGTH


    def go_to_goal(self, goal_odom_data):
        # yaw_delta = self._calculate_yaw_difference(self.current_odom_data, goal_odom_data)
        # sign = yaw_delta/abs(yaw_delta)
        # self.current_goal = goal_odom_data
        # self._start_manipulation(sign)
        move_thread = \
            threading.Thread(
                target=self._move_to_goal
            )
        self.thread_handles.append(move_thread)
        self.current_goal = goal_odom_data
        move_thread.start()

    def _move_to_goal(self):
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        pose_goal = geometry_msgs.msg.Pose()

        print(self.current_goal)
        pose_goal.orientation.x = self.current_goal.pose.orientation.x
        pose_goal.orientation.y = self.current_goal.pose.orientation.y
        pose_goal.orientation.z = self.current_goal.pose.orientation.z
        pose_goal.orientation.w = self.current_goal.pose.orientation.w
        pose_goal.position.x = self.current_goal.pose.position.x
        pose_goal.position.y = self.current_goal.pose.position.y
        pose_goal.position.z = self.current_goal.pose.position.z

        # pose_goal.orientation.x = 0
        # pose_goal.orientation.y = -1
        # pose_goal.orientation.z = 0
        # pose_goal.orientation.w = 0
        # pose_goal.position.x = 0.5
        # pose_goal.position.y = 0.5
        # pose_goal.position.z = 0.5

        # pose_goal.orientation.w = -1.0
        # pose_goal.position.x = 0.5
        # pose_goal.position.y = 0.5
        # pose_goal.position.z = 0.5

        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        print("If success: ", success)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        # if(success):
        self.move_group.clear_pose_targets()
        msg = MoveBaseActionResult()
        msg.status.status = 3
        self.current_goal = None
        self.outcome_publisher.publish(msg)

        grip_msg = MoveActionGoal()
        grip_msg.goal.width = 0.08
        grip_msg.goal.speed = 0.1
        self.gripper_publisher.publish(grip_msg)
