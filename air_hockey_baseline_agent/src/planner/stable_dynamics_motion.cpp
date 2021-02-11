/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "planner/stable_dynamics_motion.h"

#include <utility>

using namespace Eigen;
using namespace air_hockey_baseline_agent;

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
                                trajectory_msgs::MultiDOFJointTrajectory &cartTraj, double tStop) {
    xTmp_ = xCur;
    vTmp_ = vCur;

    double t_prev;
    if (cartTraj.points.size() == 0){
        t_prev = 0.;
    } else {
        t_prev = cartTraj.points.back().time_from_start.toSec();
    }

    double t = 0.;
    while (t < tStop){
        t += stepSize_;
        xTmp_ += vTmp_ * stepSize_;
        vTmp_ += stepSize_ * (-damping_.cwiseProduct(vTmp_) - stiffness_.cwiseProduct(xTmp_ - xGoal));

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
