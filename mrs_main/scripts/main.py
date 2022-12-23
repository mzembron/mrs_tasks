from plan_master.turtlebot import Turtlebot
from plan_master.manipulator import Manipulator
from plan_master.panda_manip import PandaManip
from plan_master.task_harmonizer import TaskHarmonizer

import rospy
# from nav_msgs.srv import GetPlan, GetPlanRequest

from plan_master.plan_master_class import PlanMaster

def main():
    
    rospy.init_node('mrs_master')
    rospy.sleep(2)

    plan_master = PlanMaster()
    # plan_master.subscribe(Turtlebot("/robot1"))
    # plan_master.subscribe(Turtlebot("/robot2"))
    
    plan_master.subscribe(Turtlebot("/robot1"))
    plan_master.subscribe(Turtlebot("/robot2"))
    plan_master.subscribe(Turtlebot("/robot3_dirty"))

    # manipulator = Manipulator("/robot3_manip")
    # manip_harmonizer = TaskHarmonizer("/robot3_manip")
    # manip_harmonizer.robot = manipulator
    # plan_master.subscribe(Manipulator("/robot3_manip"))
    plan_master.subscribe(PandaManip("/robot_panda"))


    rospy.spin()

if __name__ == "__main__":
    main()