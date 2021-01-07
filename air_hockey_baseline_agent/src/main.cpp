#include <ros/ros.h>

#include "agent.h"

using namespace AirHockey;

int main(int argc, char** argv){
    ros::init(argc, argv, "tactical_agent");
    ros::NodeHandle nh("/");
    Agent agent(nh, 100.0);

    agent.start();

    return 0;
}
