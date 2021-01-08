#include "planner/stable_dynamics_motion.h"

#include <utility>

using namespace AirHockey;

StableDynamicsMotion::StableDynamicsMotion(Vector2d stiffnessUpperBound, Vector2d stiffnessLowerBound,
                                           double rate, double height) :
                                     height_(height){
    stiffnessUpperBound_ = stiffnessUpperBound;
    stiffnessLowerBound_ = stiffnessLowerBound;
    stepSize_ = 1 / rate;

    viaPoint_.transforms.resize(1);
    viaPoint_.velocities.resize(1);

    setStiffness(stiffnessUpperBound);
}

StableDynamicsMotion::~StableDynamicsMotion() {

}

bool StableDynamicsMotion::plan(const Vector2d &xCur, const Vector2d &vCur, const Vector2d &xGoal,
                                trajectory_msgs::MultiDOFJointTrajectory &cartTraj, double duration) {
    xTmp_ = xCur;
    vTmp_ = vCur;

    double t = 0.;
    while (t < duration){
        t += stepSize_;
        xTmp_ += vTmp_ * stepSize_;
        vTmp_ += stepSize_ * (-damping_.cwiseProduct(vTmp_) - stiffness_.cwiseProduct(xTmp_ - xGoal));

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

void StableDynamicsMotion::scaleStiffness(const Vector2d &scale) {
    Vector2d stiffness;
    for (int i = 0; i < 2; ++i) {
        stiffness[i] = boost::algorithm::clamp(stiffness_[i] * scale[i], stiffnessLowerBound_[i], stiffnessUpperBound_[i]);
    }
    setStiffness(stiffness);
}

void StableDynamicsMotion::setStiffness(const Vector2d &stiffness) {
    stiffness_ = stiffness;
    damping_ = 2 * stiffness_.cwiseSqrt();
}

const Vector2d& StableDynamicsMotion::getStiffness() {
    return stiffness_;
}