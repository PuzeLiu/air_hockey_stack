#include <ros/ros.h>

#include "../../air_hockey_baseline_agent/include/agent.h"

using namespace AirHockey;

int main(int argc, char** argv){
    ros::init(argc, argv, "tactical_agent");
    ros::NodeHandle nh("/");
    Agent agent(nh, 100.0);

    ros::Duration(2.).sleep();

    ROS_INFO_STREAM("Go to home position");
    agent.gotoInit();
    ROS_INFO_STREAM("Start");
    while (ros::ok()){
        agent.update();
    }
    return 0;
}
