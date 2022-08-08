
# import queue

import rospy
import math
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import  Odometry
from geometry_msgs.msg import PoseStamped


def robot1_odom_callback(data):
    global robot1_position
    print("[ Robot 1 ]Odom position x: " + str(data.pose.pose.position.x -0.5))
    print("[ Robot 1s ]Odom position y: " + str(data.pose.pose.position.y - 0.5))
    robot1_position = data

def robot2_odom_callback(data):
    global robot2_position
    print("[ Robot 2 ] Odom position x: " + str(data.pose.pose.position.x -0.5))
    print("[ Robot 2 ] Odom position y: " + str(data.pose.pose.position.y - 0.5))
    robot2_position = data

def move_base_goal_callback(data):
    global pub_robot1, pub_robot2, robot1_position, robot2_position
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = 'map'
    msg.goal.target_pose.pose.position.x = data.pose.position.x
    msg.goal.target_pose.pose.position.y = data.pose.position.y

    msg.goal.target_pose.pose.orientation.x = data.pose.orientation.x
    msg.goal.target_pose.pose.orientation.y = data.pose.orientation.y
    msg.goal.target_pose.pose.orientation.z = data.pose.orientation.z
    msg.goal.target_pose.pose.orientation.w = data.pose.orientation.w

    # planing 
    
    x_goal = data.pose.position.x
    y_goal = data.pose.position.y

    x_robot1 = robot1_position.pose.pose.position.x -0.5
    y_robot1 = robot1_position.pose.pose.position.y -0.5

    x_robot2 = robot2_position.pose.pose.position.x -0.5
    y_robot2 = robot2_position.pose.pose.position.y -0.5

    distance_robot1 = math.sqrt((x_robot1-x_goal)**2 + (y_robot1-y_goal)**2)
    distance_robot2 = math.sqrt((x_robot2 - x_goal)**2 + (y_robot2-y_goal)**2)

    if(distance_robot1 > distance_robot2):
        print("publishing goal for robot2")
        pub_robot2.publish(msg)
    else:
        print("publishing goal for robot1")
        pub_robot1.publish(msg)


def main():
    global pub_robot1, pub_robot2
    rospy.init_node('mrs_master')
    rospy.sleep(2)
    rospy.Subscriber("/robot1/odom", Odometry, robot1_odom_callback)
    rospy.Subscriber("/robot2/odom", Odometry, robot2_odom_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped,move_base_goal_callback)
    pub_robot1 = rospy.Publisher('/robot1/move_base/goal',MoveBaseActionGoal, queue_size=10)
    pub_robot2 = rospy.Publisher('/robot2/move_base/goal',MoveBaseActionGoal, queue_size=10)
    rospy.sleep(2)
    msg = MoveBaseActionGoal()
    # msg.goal.target_pose.header.frame_id = 'map'
    # msg.goal.target_pose.pose.position.x = -5.0
    # msg.goal.target_pose.pose.position.y = 4.0
    # msg.goal.target_pose.pose.orientation.w = 1.0
    # print("publishing goal for robot1")
    # pub.publish(msg)

    rospy.spin()



if __name__ == '__main__':
    main()