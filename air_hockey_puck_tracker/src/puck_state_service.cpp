#include "ros/ros.h"
#include "air_hockey_puck_tracker/puck_state_service.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_puck_state");
    ros::NodeHandle n;
    PuckStateHandler psh(n);
    ros::ServiceServer service = n.advertiseService("get_puck_state", &PuckStateHandler::getPuckState, &psh);
    ROS_INFO("Ready to call service.");
    ros::spin();

    return 0;
}
