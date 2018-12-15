#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

...

tf::TransformListener tf(ros::Duration(10));
costmap_2d::Costmap2DROS costmap("my_costmap", tf);
