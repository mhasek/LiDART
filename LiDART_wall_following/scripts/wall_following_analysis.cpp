#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "lidart_wall_following/wall_following_analysis.h"
#include <cmath>

int n = 0;
double sum = 0.0;
double avg_abs_error = 0.0;
double max_abs_error = 0.0;

void errorCallback(const std_msgs::Float64 error_msg)
{
  sum += fabs(error_msg.data);
  n++;
  avg_abs_error = sum / n;
  if (max_abs_error < error_msg.data){
    max_abs_error = error_msg.data;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidart_analysis");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("pid_error", 1000, errorCallback);
  ros::Publisher pub = n.advertise
    <lidart_wall_following::wall_following_analysis>("wall_following_analysis", 1000);
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    //ROS_INFO("%f", avg_abs_error);
    lidart_wall_following::wall_following_analysis msg;
    msg.avg_abs_error = avg_abs_error;
    msg.max_abs_error = max_abs_error;
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}