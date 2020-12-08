//
// Created by puze on 06.12.20.
//

#include <chrono>
#include <iostream>

#include "null_space_opt.h"
#include "null_space_opt_ros.h"

using namespace std;
using namespace null_space_optimization;
using namespace iiwas_kinematics;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "null_space_optimizer");

    Vector3d tcp_ee(0., 0., 0.5);
    Quaterniond tcpQuat(1., 0., 0., 0.);

    Kinematics kinematics(tcp_ee, tcpQuat);

    NullSpaceOptimizerROS optimizerRos(kinematics);

    Kinematics::JointArrayType q_cur;

    while (ros::ok()){
        optimizerRos.update();
    }

    return 0;
}