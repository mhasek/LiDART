#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <occupancy_grid/local_rrt_result.h>
#include <occupancy_grid/all_waypoints.h>

#include <cmath>

visualization_msgs::Marker points, line_strip;
visualization_msgs::Marker points_wp, line_strip_wp;

void vizCallBack(const occupancy_grid::local_rrt_result::ConstPtr& data){
  line_strip.points.clear();
  for (int i = 0; i < sizeof(data->global_path_x) / sizeof(data->global_path_x[0]); i++){
  	geometry_msgs::Point pt;
	pt.x = data->global_path_x[i];
	pt.y = data->global_path_y[i];
  	points.points.push_back(pt);
  	line_strip.points.push_back(pt);
  }
}

void wpCallBack(const occupancy_grid::all_waypoints::ConstPtr& data){
  line_strip_wp.points.clear();
  for (int i = 0; i < sizeof(data->waypoints_x) / sizeof(data->waypoints_x[0]); i++){
  	geometry_msgs::Point pt;
	pt.x = data->waypoints_x[i];
	pt.y = data->waypoints_y[i];
  	points_wp.points.push_back(pt);
  	line_strip_wp.points.push_back(pt);
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "local_path_visualizer");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("local_rrt_path_viz", 10);
  ros::Subscriber sub = n.subscribe("local_rrt_result", 1000, vizCallBack);
  ros::Publisher marker_pub_wp = n.advertise<visualization_msgs::Marker>("all_waypoints_viz", 10);
  ros::Subscriber sub_wp = n.subscribe("all_waypoints", 1000, wpCallBack);

  points.header.frame_id = line_strip.header.frame_id = "/map";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "local_rrt_path_viz";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points_wp.header.frame_id = line_strip_wp.header.frame_id = "/map";
  points_wp.header.stamp = line_strip_wp.header.stamp = ros::Time::now();
  points_wp.ns = line_strip_wp.ns = "local_rrt_path_viz";
  points_wp.action = line_strip_wp.action = visualization_msgs::Marker::ADD;
  points_wp.pose.orientation.w = line_strip_wp.pose.orientation.w = 1.0;

  ros::Rate r(30);

  points.id = 0;
  line_strip.id = 1;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  points_wp.id = 0;
  line_strip_wp.id = 1;
  points_wp.type = visualization_msgs::Marker::POINTS;
  line_strip_wp.type = visualization_msgs::Marker::LINE_STRIP;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points_wp.scale.x = 0.1;
  points_wp.scale.y = 0.1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  line_strip_wp.scale.x = 0.1;

  // Points are green & red
  points.color.g = 1.0f;
  points.color.a = 1.0;
  points_wp.color.r = 1.0f;
  points_wp.color.a = 1.0;

  // Line strip is blue & yellow
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  line_strip_wp.color.r = 1.0f;
  line_strip_wp.color.g = 1.0f;
  line_strip_wp.color.a = 1.0;

  float f = 0.0;
  while (ros::ok())
  {

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

	marker_pub_wp.publish(points_wp);
    marker_pub_wp.publish(line_strip_wp);

    ros::spinOnce();
    r.sleep();
    
  }
}
