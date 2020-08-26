#ifndef SAARTI_NODE_H
#define SAARTI_NODE_H

// ros
#include "ros/ros.h"
#include "ros/time.h"
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/Path.h"

// external libs
#include "dubins.h"
#include "planning_util.h"

// misc
#include <stdio.h>
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant" // supress warning at ros prints

// namespaces
using std::cout;
using std::endl;

namespace kth_forestplanner_node{

class ForestPlanner{

public:
    ForestPlanner(ros::NodeHandle nh);

private:
    ros::NodeHandle nh_;
    double dt;
    ros::Publisher path_pub_;

    // msg variables
    cont::pathstruct path_;

    // functions
    nav_msgs::Path pathstruct2rospath(cont::pathstruct);

};

} // end namespace saarti_node

#endif // SAARTI_NODE_H
