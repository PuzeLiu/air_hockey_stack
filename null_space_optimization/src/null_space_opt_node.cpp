//
// Created by puze on 06.12.20.
//

#include <chrono>
#include <iostream>

#include "null_space_opt.h"
#include "null_space_opt_ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace null_space_optimization;
using namespace iiwas_kinematics;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "null_space_optimizer");
    ros::NodeHandle nh;
    ros::Rate rate(1000);

    bool closeLoop = false;

    Vector3d tcp_ee(0., 0., 0.5);
    Quaterniond tcpQuat(1., 0., 0., 0.);

    Kinematics kinematics(tcp_ee, tcpQuat);

    NullSpaceOptimizerROS optimizerRos(kinematics);

    ros::Subscriber cartPosSub, jointPosSub;

    if (!closeLoop) {
        cartPosSub = nh.subscribe("cartesian_command", 1, &NullSpaceOptimizerROS::cartesianCmdCallback, &optimizerRos);
        jointPosSub = nh.subscribe("joint_state", 1, &NullSpaceOptimizerROS::jointStateCallback, &optimizerRos);
    } else {
        message_filters::Subscriber<CartersianTrajectory> cartPosSyncSub(nh, "cartesian_command", 1);
        message_filters::Subscriber<sensor_msgs::JointState> jointPosSyncSub(nh, "joint_state", 1);
        message_filters::TimeSynchronizer<CartersianTrajectory, sensor_msgs::JointState> msgSynchronizer(cartPosSyncSub, jointPosSyncSub, 1);
        msgSynchronizer.registerCallback(&NullSpaceOptimizerROS::cmdCallback, &optimizerRos);
    }

    Kinematics::JointArrayType q_cur;

    while (ros::ok()){
        optimizerRos.update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}