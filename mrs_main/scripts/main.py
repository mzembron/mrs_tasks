from plan_master.turtlebot import Turtlebot
from plan_master.manipulator import Manipulator
from plan_master.robot_task_harmonizer import RobotTaskHarmonizer

import rospy
# from nav_msgs.srv import GetPlan, GetPlanRequest

from plan_master.plan_master_class import PlanMaster

def main():
    
    rospy.init_node('mrs_master')
    rospy.sleep(2)

    plan_master = PlanMaster()
    # plan_master.subscribe(Turtlebot("/robot1"))
    # plan_master.subscribe(Turtlebot("/robot2"))

    plan_master.subscribe(RobotTaskHarmonizer("/robot1"))
    plan_master.subscribe(RobotTaskHarmonizer("/robot2"))
    manipulator = Manipulator("/robot3_manip")
    manip_harmonizer = RobotTaskHarmonizer("/robot3_manip")
    manip_harmonizer.robot = manipulator
    plan_master.subscribe(manip_harmonizer)


    rospy.spin()

if __name__ == "__main__":
    main()