#include "planner/cubic_linear_motion.h"

using namespace AirHockey;

CubicLinearMotion::CubicLinearMotion(double rate, double height) {
    stepSize_ = 1 / rate;
    height_ = height;

    viaPoint_.transforms.resize(1);
    viaPoint_.velocities.resize(1);
}

CubicLinearMotion::~CubicLinearMotion() {

}

bool
CubicLinearMotion::plan(const Vector2d& pStart, const Vector2d& vStart, const Vector2d& pStop, const Vector2d& vStop, double tStop,
                                   trajectory_msgs::MultiDOFJointTrajectory &cartTraj) {
    coefficients_.col(0) = pStart;
    coefficients_.col(1) = vStart;
    coefficients_.col(2) = (-3.0 * pStart + 3.0 * pStop - 2.0 * vStart * tStop - vStop * tStop) / pow(tStop, 2);
    coefficients_.col(3) = (2.0 * pStart - 2.0 * pStop + vStart * tStop + vStop * tStop) / pow(tStop, 3);

    double t = 0.;
    while (t <= tStop) {
        t += stepSize_;
//        xTmp_ = coefficients_.col(0);
        xTmp_ = coefficients_.col(0) + coefficients_.col(1) * t + coefficients_.col(2) * pow(t, 2) + coefficients_.col(3) * pow(t, 3);
        vTmp_ = coefficients_.col(1) + 2 * coefficients_.col(2) * t + 3 * coefficients_.col(3) * pow(t, 2);

        viaPoint_.transforms[0].translation.x = xTmp_[0];
        viaPoint_.transforms[0].translation.y = xTmp_[1];
        viaPoint_.transforms[0].translation.z = height_;
        viaPoint_.velocities[0].linear.x = vTmp_[0];
        viaPoint_.velocities[0].linear.y = vTmp_[1];
        viaPoint_.velocities[0].linear.z = 0.0;
        viaPoint_.time_from_start = ros::Duration(t);
        cartTraj.points.push_back(viaPoint_);
    }
    cartTraj.header.stamp = ros::Time::now();
    return true;
}