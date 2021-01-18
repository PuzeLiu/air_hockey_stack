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
    Matrix<double, 2, 4> coefficients;
    coefficients.col(0) = pStart;
    coefficients.col(1) = vStart;
    coefficients.col(2) = (-3.0 * pStart + 3.0 * pStop - 2.0 * vStart * tStop - vStop * tStop) / pow(tStop, 2);
    coefficients.col(3) = (2.0 * pStart - 2.0 * pStop + vStart * tStop + vStop * tStop) / pow(tStop, 3);

    double t_prev;
    if (cartTraj.points.size() == 0){
        t_prev = 0.;
    } else {
        t_prev = cartTraj.points.back().time_from_start.toSec();
    }
    double t = 0.;
    Vector2d xTmp_, vTmp_;
    while (t <= tStop) {
        t += stepSize_;
        xTmp_ = coefficients.col(0) + coefficients.col(1) * t + coefficients.col(2) * pow(t, 2) + coefficients.col(3) * pow(t, 3);
        vTmp_ = coefficients.col(1) + 2 * coefficients.col(2) * t + 3 * coefficients.col(3) * pow(t, 2);

        viaPoint_.transforms[0].translation.x = xTmp_[0];
        viaPoint_.transforms[0].translation.y = xTmp_[1];
        viaPoint_.transforms[0].translation.z = height_;
        viaPoint_.velocities[0].linear.x = vTmp_[0];
        viaPoint_.velocities[0].linear.y = vTmp_[1];
        viaPoint_.velocities[0].linear.z = 0.0;
        viaPoint_.time_from_start = ros::Duration(t + t_prev);
        cartTraj.points.push_back(viaPoint_);
    }
    cartTraj.header.stamp = ros::Time::now();
    return true;
}

bool CubicLinearMotion::plan(const Vector3d &pStart, const Vector3d &vStart, const Vector3d &pStop,
                             const Vector3d &vStop, double tStop, trajectory_msgs::MultiDOFJointTrajectory &cartTraj) {
    Matrix<double, 3, 4> coefficients;
    coefficients.col(0) = pStart;
    coefficients.col(1) = vStart;
    coefficients.col(2) = (-3.0 * pStart + 3.0 * pStop - 2.0 * vStart * tStop - vStop * tStop) / pow(tStop, 2);
    coefficients.col(3) = (2.0 * pStart - 2.0 * pStop + vStart * tStop + vStop * tStop) / pow(tStop, 3);

    double t_prev;
    if (cartTraj.points.size() == 0){
        t_prev = 0.;
    } else {
        t_prev = cartTraj.points.back().time_from_start.toSec();
    }
    double t = 0.;
    Vector3d xTmp_, vTmp_;
    while (t <= tStop) {
        t += stepSize_;
        xTmp_ = coefficients.col(0) + coefficients.col(1) * t + coefficients.col(2) * pow(t, 2) + coefficients.col(3) * pow(t, 3);
        vTmp_ = coefficients.col(1) + 2 * coefficients.col(2) * t + 3 * coefficients.col(3) * pow(t, 2);

        viaPoint_.transforms[0].translation.x = xTmp_[0];
        viaPoint_.transforms[0].translation.y = xTmp_[1];
        viaPoint_.transforms[0].translation.z = xTmp_[2];
        viaPoint_.velocities[0].linear.x = vTmp_[0];
        viaPoint_.velocities[0].linear.y = vTmp_[1];
        viaPoint_.velocities[0].linear.z = vTmp_[2];
        viaPoint_.time_from_start = ros::Duration(t + t_prev);
        cartTraj.points.push_back(viaPoint_);
    }
    cartTraj.header.stamp = ros::Time::now();
    return true;
}