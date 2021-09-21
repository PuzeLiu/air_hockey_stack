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


#ifndef SRC_STABLE_DYNAMICS_MOTION_H
#define SRC_STABLE_DYNAMICS_MOTION_H

#include <Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <boost/algorithm/clamp.hpp>


namespace air_hockey_baseline_agent{
class StableDynamicsMotion {
public:
    StableDynamicsMotion(Eigen::Vector2d stiffnessUpperBound, Eigen::Vector2d stiffnessLowerBound, double rate, double height);
    ~StableDynamicsMotion();

    bool plan(const Eigen::Vector2d& xCur, const Eigen::Vector2d& vCur, const Eigen::Vector2d& xGoal,
              trajectory_msgs::MultiDOFJointTrajectory& cartTraj, double tStop = 0.1);

    void scaleStiffness(const Eigen::Vector2d& scale);
    void setStiffness(const Eigen::Vector2d& stiffness);
    const Eigen::Vector2d& getStiffness();

private:
    Eigen::Vector2d stiffnessUpperBound_, stiffnessLowerBound_;
    Eigen::Vector2d stiffness_, damping_;
    double stepSize_;
    double height_;

    Eigen::Vector2d xTmp_, vTmp_;

    trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;
};
}

#endif //SRC_STABLE_DYNAMICS_MOTION_H
