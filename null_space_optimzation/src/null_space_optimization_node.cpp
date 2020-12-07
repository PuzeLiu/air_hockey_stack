//
// Created by puze on 06.12.20.
//

#include <chrono>
#include <iostream>
#include <ros/ros.h>

#include "null_space_opt.h"

using namespace std;
using namespace null_space_optim;
using namespace iiwas_kinematics;

int main(int argc, char *argv[]) {
    Vector3d tcp_ee(0., 0., 0.5);
    Quaterniond tcp_quat(1., 0., 0., 0.);

    Kinematics kinematics(tcp_ee, tcp_quat);
    NullSpaceOptimizer optimizer(kinematics);

    Kinematics::JointArrayType q_cur;
    Kinematics::JacobianPosType jac_pos;

    Vector3d x_des, dx_des;
    Quaterniond quat_cur;
    x_des.setRandom();
    dx_des.setZero();
    Kinematics::JointArrayType weights;
    weights << 40., 40., 20., 40., 10., 10., 10.;
    auto start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q_cur.setRandom();
        q_cur *=  M_PI;
        kinematics.ForwardKinematics(q_cur, x_des, quat_cur);
        dx_des = dx_des.setRandom();
        dx_des = dx_des * 0.01;
        x_des += dx_des;
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Jacobian Null Space Optimization Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";
    return 0;
}