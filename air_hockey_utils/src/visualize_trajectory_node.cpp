#include "air_hockey_utils/trajectory_visualizer.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "air_hockey_trajectory_visualizer");

    ROS_INFO_STREAM(ros::this_node::getNamespace());
    ros::NodeHandle nh(ros::this_node::getNamespace());
    ros::NodeHandle nhl("~");
    double update_frequency = 10;
    nhl.getParam("update_frequency", update_frequency);
    int buffer_size;
    nhl.getParam("buffer_size", buffer_size);
    TrajectoryVisualizer trajectoryVisualizer(nh, buffer_size);

    ros::Rate rate(update_frequency);
    ros::Duration(1.0).sleep();
    while (ros::ok()){
        ros::spinOnce();
        trajectoryVisualizer.update();
        rate.sleep();
    }

    return 0;
}