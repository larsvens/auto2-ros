#include "kth_forestplanner/kth_forestplanner.h"


namespace kth_forestplanner_node {

ForestPlanner::ForestPlanner(ros::NodeHandle nh){
    nh_ = nh;
    dt = 0.2;
    ros::Rate loop_rate(1/dt);

    // pubs & subs
    path_pub_ = nh.advertise<nav_msgs::Path>("path",1);

    // wait until gridmap is received (temporarily disabled)
    bool gridmap_received = true;
    while( !gridmap_received ){
        ROS_INFO_STREAM("waiting for gridmap");
        ros::spinOnce();
        loop_rate.sleep();
    }

    // main loop
    while (ros::ok()){
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("main_ loop_");

        // define initial and goal poses
        double q0[] = { 0,0,0 };
        //double q1[] = { 4,4,3.142 };
        double q1[] = { 6,4,0 };

        // define path properties
        double ds = 0.5;
        double turning_radius = 3.0;

        // compute path
        DubinsPath dubinspath;
        dubins_shortest_path( &dubinspath, q0, q1, turning_radius);
        cont::pathstruct p;
        dubins_get_sampled_path(&dubinspath,ds,p);

        ROS_INFO_STREAM("Computing single Dubins path of length " << p.s.size() * ds << "m ");

        // build ros message
        nav_msgs::Path path_msg = ForestPlanner::pathstruct2rospath(p);

        // publish
        path_pub_.publish(path_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/*
 * FUNCTIONS
 */

nav_msgs::Path ForestPlanner::pathstruct2rospath(cont::pathstruct ps){
    nav_msgs::Path rp;
    std::string frame_id = "base_link";
    rp.header.stamp = ros::Time::now();
    rp.header.frame_id = frame_id;
    for (uint i=0;i<ps.s.size();i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;
        pose.pose.position.x = double(ps.X.at(i));
        pose.pose.position.y = double(ps.Y.at(i));
        tf2::Quaternion q;
        q.setRPY(0,0,double(ps.psi.at(i)));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        rp.poses.push_back(pose);
    }
    return rp;
}


} // end namespace kth_forestplanner_node

int main(int argc, char **argv){
    ros::init(argc, argv, "forestplanner");
    ros::NodeHandle nh;
    kth_forestplanner_node::ForestPlanner fp(nh);
    return 0;
}
