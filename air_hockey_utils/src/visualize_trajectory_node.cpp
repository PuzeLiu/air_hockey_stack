#include "air_hockey_utils/trajectory_visualizer.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "air_hockey_trajectory_visualizer");

    ROS_INFO_STREAM(ros::this_node::getNamespace());
    ros::NodeHandle nh(ros::this_node::getNamespace());
    ros::NodeHandle nhl("~");
    double update_frequency = 10;
    nhl.getParam("update_frequency", update_frequency);
    int des_buffer_size, act_buffer_size;
    nhl.getParam("desired_buffer_size", des_buffer_size);
	nhl.getParam("actual_buffer_size", act_buffer_size);
    TrajectoryVisualizer trajectoryVisualizer(nh, des_buffer_size, act_buffer_size);

    ros::Rate rate(update_frequency);
    ros::Duration(2.0).sleep();
    while (ros::ok()){
        ros::spinOnce();
        trajectoryVisualizer.update();
        rate.sleep();
    }

    return 0;
}