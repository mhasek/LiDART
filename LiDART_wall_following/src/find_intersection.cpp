#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <vector>
#include "lidart_gap_finding/gaps.h"
#include "lidart_gap_finding/gap.h"

int gaps_cnt = 0;
int prev_gaps_cnt = 0;

int counter_hallway = 0;
int counter_intersect = 0;
int thres_hallway = 10; //0.25s
int thres_intersect = 1; //0.125s

bool curr = false;
std_msgs::Bool curr_msg;

void gapsCallback(const lidart_gap_finding::gaps::ConstPtr& gaps_data)
{
  // std::vector<lidart_gap_finding::gap> gaps_data_vec = static_cast< std::vector<lidart_gap_finding::gap> > (gaps_data);
  prev_gaps_cnt = gaps_cnt;
  gaps_cnt = gaps_data->data.size();

  if (gaps_cnt > 1){
    if (prev_gaps_cnt == 1) counter_hallway = 0;
    ++counter_intersect;
  } else {
    if (prev_gaps_cnt > 1) counter_intersect = 0;
    ++counter_hallway;
  }

  if (curr == true && counter_hallway > thres_hallway){
    curr = false;
  } else if (curr == false && counter_intersect > thres_intersect){
    curr = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_intersection_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gaps_data", 1000, gapsCallback);
  ros::Publisher pub = n.advertise<std_msgs::Bool>("intersection", 1000);
  ros::Rate loop_rate(5);

  while (ros::ok())
  { 
    //pub.publish(msg);
    curr_msg.data = curr;
    ROS_INFO("%d",gaps_cnt);
    pub.publish(curr_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}