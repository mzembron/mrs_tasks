import queue
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import  Odometry
from geometry_msgs.msg import PoseStamped

def robot1_odom_callback(data):
    print("Odom position x: " + str(data.pose.pose.position.x -0.5))
    print("Odom position y: " + str(data.pose.pose.position.y - 0.5))

def move_base_goal_callback(data):
    global pub 
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = 'map'
    msg.goal.target_pose.pose.position.x = data.pose.position.x
    msg.goal.target_pose.pose.position.y = data.pose.position.y
    msg.goal.target_pose.pose.orientation.w = 1.0
    print("publishing goal for robot1")
    pub.publish(msg)

def main():
    global pub
    rospy.init_node('mrs_master')
    rospy.sleep(2)
    rospy.Subscriber("/robot1/odom", Odometry, robot1_odom_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped,move_base_goal_callback)
    pub = rospy.Publisher('/robot1/move_base/goal',MoveBaseActionGoal, queue_size=10)
    rospy.sleep(2)
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = 'map'
    msg.goal.target_pose.pose.position.x = -5.0
    msg.goal.target_pose.pose.position.y = 4.0
    msg.goal.target_pose.pose.orientation.w = 1.0
    print("publishing goal for robot1")
    pub.publish(msg)

    rospy.spin()



if __name__ == '__main__':
    main()