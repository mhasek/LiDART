#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <cmath>

visualization_msgs::Marker points, line_strip;
visualization_msgs::Marker points_odom, line_strip_odom;


void vizCallBack(const geometry_msgs::Point::ConstPtr& viz_data){
  points.points.push_back(*viz_data);
}

void vizOdomCallBack(const geometry_msgs::Point::ConstPtr& viz_odom_data){
  points_odom.points.push_back(*viz_odom_data);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "scan_matcher_visualization_2");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("scan_matcher_visualization", 10);
  ros::Subscriber sub = n.subscribe("scan_match_location", 1000, vizCallBack);
  ros::Publisher marker_pub_odom = n.advertise<visualization_msgs::Marker>("scan_matcher_odom_visualization", 10);
  ros::Subscriber sub_odom = n.subscribe("scan_match_corr_odom_pos", 1000, vizOdomCallBack);

  points.header.frame_id = line_strip.header.frame_id = "/map";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "scan_matcher_visualization_2";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points_odom.header.frame_id = line_strip_odom.header.frame_id = "/map";
  points_odom.header.stamp = line_strip_odom.header.stamp = ros::Time::now();
  points_odom.ns = line_strip_odom.ns = "scan_matcher_visualization_2";
  points_odom.action = line_strip_odom.action = visualization_msgs::Marker::ADD;
  points_odom.pose.orientation.w = line_strip_odom.pose.orientation.w = 1.0;

  ros::Rate r(30);

  points.id = 0;
  line_strip.id = 1;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  points_odom.id = 0;
  line_strip_odom.id = 1;
  points_odom.type = visualization_msgs::Marker::POINTS;
  line_strip_odom.type = visualization_msgs::Marker::LINE_STRIP;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points_odom.scale.x = 0.1;
  points_odom.scale.y = 0.1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  line_strip_odom.scale.x = 0.1;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Points odom are red
  points_odom.color.r = 1.0f;
  points_odom.color.g = 0.0f;
  points_odom.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line strip odom is purple (?)
  line_strip_odom.color.b = 1.0;
  line_strip_odom.color.r = 1.0;
  line_strip_odom.color.a = 1.0;

  float f = 0.0;
  while (ros::ok())
  {

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    marker_pub_odom.publish(points_odom);
    marker_pub_odom.publish(line_strip_odom);

    ros::spinOnce();
    r.sleep();
    
  }
}
