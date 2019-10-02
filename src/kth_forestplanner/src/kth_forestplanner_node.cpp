#include "kth_forestplanner/kth_forestplanner.h"

int printConfiguration(double q[3], double x, void* user_data) {
    printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    return 0;
}

namespace kth_forestplanner_node {

ForestPlanner::ForestPlanner(ros::NodeHandle nh){
    nh_ = nh;
    dt = 0.2;
    ros::Rate loop_rate(1/dt);

    // pubs & subs

    // visualization msgs

    // wait until gridmap is received
    bool gridmap_received = true; // tmp!
    while( !gridmap_received ){
        ROS_INFO_STREAM("waiting for gridmap");
        ros::spinOnce();
        loop_rate.sleep();
    }

    // main loop
    while (ros::ok()){
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("main_ loop_");

        double q0[] = { 0,0,0 };
        double q1[] = { 4,4,3.142 };
        double turning_radius = 3.0;
        DubinsPath path;
        dubins_shortest_path( &path, q0, q1, turning_radius);
        dubins_path_sample_many( &path, 0.5, printConfiguration, NULL);

        //std::cout << "qi:   "<< path.qi << std::endl;
        //std::cout << "rho:  "<< path.rho << std::endl;
        //std::cout << "type: "<< path.type << std::endl;
        //std::cout << "param:"<< path.param << std::endl;

        ros::spinOnce();
        loop_rate.sleep();

    }

}

// function



}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "forestplanner");
    ros::NodeHandle nh;
    kth_forestplanner_node::ForestPlanner fp(nh);
    return 0;
}
