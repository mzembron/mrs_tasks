from plan_master.turtlebot import Turtlebot

import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest

from plan_master.plan_master_class import PlanMaster

def main():
    rospy.init_node('mrs_master')
    rospy.sleep(2)
    # robot1 = Turtlebot("/robot1/odom", '/robot1/move_base/goal' )

    ## Requesing plan exmple 
    make_plan = rospy.ServiceProxy('/robot1/move_base_node/NavfnROS/make_plan', GetPlan)
    req = GetPlanRequest()


    plan_master = PlanMaster()
    plan_master.subscribe(Turtlebot("/robot1"))
    plan_master.subscribe(Turtlebot("/robot2"))


    rospy.spin()

if __name__ == "__main__":
    main()