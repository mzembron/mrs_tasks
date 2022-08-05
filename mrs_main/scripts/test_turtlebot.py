from Turtlebot import Turtlebot

import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest

def main():
    rospy.init_node('mrs_master')
    rospy.sleep(2)
    # robot1 = Turtlebot("/robot1/odom", '/robot1/move_base/goal' )

    ## Requesing plan exmple 
    make_plan = rospy.ServiceProxy('/robot1/move_base_node/NavfnROS/make_plan', GetPlan)
    req = GetPlanRequest()

    req.start.header.frame_id = 'map'
    req.start.pose.position.x = 1.34
    req.start.pose.position.y = 0.37
    req.start.pose.orientation.w = 1

    req.goal.header.frame_id = 'map'
    req.goal.pose.position.x = -4
    req.goal.pose.position.y = 1
    req.goal.pose.orientation.w = 1

    req.tolerance = 0.1
    res = make_plan(req)
    print(res)
    rospy.spin()

if __name__ == "__main__":
    main()