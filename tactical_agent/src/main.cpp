//
// Created by puze on 27.12.20.
//
#include <ros/ros.h>

#include "agent.h"

using namespace tactical_agent;

int main(int argc, char** argv){
    ros::init(argc, argv, "tactical_agent");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    Agent agent=Agent();

    ros::Duration(1.).sleep();

    while (ros::ok()){
        agent.update();
        rate.sleep();
    }
    return 0;
}
