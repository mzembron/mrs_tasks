import roslib
import time
import rospy
import actionlib


from mrs_msgs.msg import TaskRequestAction, TaskRequestGoal

if __name__ == '__main__':
    rospy.init_node('order_task_client')
    client = actionlib.SimpleActionClient('plan_master/order_task', TaskRequestAction)
    client.wait_for_server()

    goal = TaskRequestGoal()
    # Fill in the goal here
    goal.task_description.type = 'GT'
    goal.task_description.data.append('hall')
    client.send_goal(goal)
    print(client.wait_for_result())
    print(client.feedback_cb())
    print(client.get_result())