#include <pluginlib/class_list_macros.h>
#include "rrt_planner.h"
#include <ros/ros.h>
#include "lidart_2dnav/GetPath.h"
#include "std_msgs/Int64.h"
#include "lidart_2dnav/Path.h"
#include <cstdlib>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_planner {

bool initialized_ = false;
ros::ServiceClient client;
lidart_2dnav::GetPath srv;

GlobalPlanner::GlobalPlanner (){

}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  initialize(name, costmap_ros);
}


void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
    // ros::init(argc, argv);
    // client = n.serviceClient<lidart_2dnav::GetPath>('get_path_points');
    initialized_ = true;
  }
// ADD
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
  std_msgs::Int64 number_of_laps;
  number_of_laps.data = 1;
  srv.request.start_point = start;
  srv.request.num_of_laps = number_of_laps.data;
  if (ros::service::call("get_path_points", srv)) {
    plan = srv.response.path.poses;
    ROS_INFO("SUCCESS");
  }
  else {
    ROS_INFO("FAIL");
  }
 return true;
}


};
