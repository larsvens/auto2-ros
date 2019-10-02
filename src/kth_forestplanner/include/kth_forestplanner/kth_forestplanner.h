#ifndef SAARTI_NODE_H
#define SAARTI_NODE_H

#include "ros/ros.h"
#include "ros/time.h"
#include "nav_msgs/Path.h"

// external libs
#include "dubins.h"

// misc
#include <stdio.h>
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant" // supress warning at ros prints


#endif // SAARTI_NODE_H

namespace kth_forestplanner_node{

class ForestPlanner{

public:
    ForestPlanner(ros::NodeHandle nh);

private:
    ros::NodeHandle nh_;
    double dt;
};

} // end namespace saarti_node

