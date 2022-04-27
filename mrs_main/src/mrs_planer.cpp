#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "global_planner/planner_core.h"
#include <math.h> 
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include "dwa_local_planner/dwa_planner_ros.h"
#include <dwa_local_planner/DWAPlannerConfig.h>
#include <rotate_recovery/rotate_recovery.h>


geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped currentOdomPose;
geometry_msgs::Twist currentOdomTwist;
bool is_goal = false;
bool planToBeMade = false;
std::vector<geometry_msgs::PoseStamped> globalPlan;




void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  currentOdomPose.pose = msg->pose.pose;
  currentOdomPose.header = msg->header;
  currentOdomTwist = msg->twist.twist;

  // ROS_INFO("Current position: x: %f, y: %f",currentOdomPose.pose.position.x, currentOdomPose.pose.position.y);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goal = *msg;
  is_goal = true;
  planToBeMade = true;
  goal.header.frame_id = "map";

  ROS_INFO("Goal x: %f, y: %f",goal.pose.position.x, goal.pose.position.y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrs_planer");
  float distance = 0;


  ros::NodeHandle n;
  ros::Rate rate(10.0);


  tf2_ros::Buffer buffer(ros::Duration(10)); 
  tf2_ros::TransformListener tfListener(buffer);

  costmap_2d::Costmap2DROS globalCostmap("global_costmap", buffer);
  costmap_2d::Costmap2DROS localCostmap("local_costmap", buffer);

  global_planner::GlobalPlanner steroGlobalPlanner("global_planer", globalCostmap.getCostmap(), "map");

  base_local_planner::TrajectoryPlannerROS tp;
  tp.initialize("local_planer", &buffer, &localCostmap);

  rotate_recovery::RotateRecovery rr;
  rr.initialize("recovery_behaviour", &buffer, &globalCostmap, &localCostmap);

  ros::Subscriber odomSubscriber = n.subscribe("/robot1_tf/odom", 1000, odomCallback);
  ros::Subscriber goalSubscriber = n.subscribe("/move_base_simple/goal", 1000, goalCallback);
  ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 1000); //maybe robot1/cmd_vel
  ros::Publisher distancePublisher = n.advertise<std_msgs::String>("mrs_robot1_distance_to_goal", 1000);


  while (ros::ok())
  {
    if (planToBeMade)
    {
      currentOdomPose.header.frame_id = "map";
      steroGlobalPlanner.makePlan(currentOdomPose, goal, globalPlan);
  
      //steroGlobalPlanner.publishPlan(globalPlan); // nie potrzebny
      planToBeMade= false;
    }

    if (is_goal)
    {
      std::stringstream ss;
      std_msgs::String msg;
      tp.setPlan(globalPlan);
      bool  valid_trajectory_was_found= tp.computeVelocityCommands(currentOdomTwist);
      //tp.dwaComputeVelocityCommands(currentOdomPose, currentOdomTwist) ??

      if (valid_trajectory_was_found)
      {
        velocityPublisher.publish(currentOdomTwist);
        is_goal= !tp.isGoalReached();
      }
      else{
        ROS_INFO("Runnning recovery behaviour");
        localCostmap.resetLayers();
        currentOdomTwist.angular.z = 1;
        velocityPublisher.publish(currentOdomTwist);
        // rr.runBehavior();
                ss << "Goal reached!";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());

      }

      if(!is_goal){
        // is_goal zwraca false jesli nie ma celu
        //  w tej petli oznacza to ze cel wlasnie zostal osiagniety
        ss << "Goal reached!";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
      }
      else{
        distance = sqrt(pow((currentOdomPose.pose.position.x - goal.pose.position.x),2)+pow((currentOdomPose.pose.position.y - goal.pose.position.y),2));
        char message[100];
        sprintf(message, "Distance from goal:  %f", distance);
        ss << message;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
      }
      
      distancePublisher.publish(msg);
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}