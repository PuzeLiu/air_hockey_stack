//
// Created by airhockey on 03.08.21.
//

#include <ros/ros.h>
#include <thread>

#include "air_hockey_puck_tracker/PuckTracker.hpp"


using namespace air_hockey_baseline_agent;
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "puck_tracker_wrapper");
    ros::NodeHandle nh("/");
    ROS_INFO_STREAM("Puck Tracker Wrapper");

    PuckTracker puckTracker(nh, 0.0);
    puckTracker.provideServices();

    ros::spin();
    nh.shutdown();
    return 0;
}

