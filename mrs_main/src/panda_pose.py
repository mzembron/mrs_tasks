from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from math import pi, tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list

## BEGIN_SUB_TUTORIAL setup
##
## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
## kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

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

## END_SUB_TUTORIAL

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
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")
## END_SUB_TUTORIAL

# Misc variables
# self.box_name = ""
# self.robot = robot
# self.scene = scene
# self.move_group = move_group
# self.display_trajectory_publisher = display_trajectory_publisher
# self.planning_frame = planning_frame
# self.eef_link = eef_link
# self.group_names = group_names

## BEGIN_SUB_TUTORIAL plan_to_pose
##
## Planning to a Pose Goal
## ^^^^^^^^^^^^^^^^^^^^^^^
## We can plan a motion for this group to a desired pose for the
## end-effector:
pose_goal = geometry_msgs.msg.Pose()

pose_goal.orientation.x = 0.001
pose_goal.orientation.y = -0.001
pose_goal.orientation.z = -0.732
pose_goal.orientation.w = -0.68
pose_goal.position.x = -7
pose_goal.position.y = 0
pose_goal.position.z = 0.5

move_group.set_pose_target(pose_goal)
success = move_group.go(wait=True)
print("If success: ", success)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
move_group.clear_pose_targets()

