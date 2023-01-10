import roslib
import time
import rospy
import actionlib


from mrs_msgs.msg import TaskRequestAction, TaskRequestGoal

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('do_dishes', TaskRequestAction)
    client.wait_for_server()

    goal = TaskRequestGoal()
    # Fill in the goal here
    goal.task_description.type = "GT"
    goal.task_description.data.append('kitchen')
    client.send_goal(goal)
    print(client.wait_for_result())