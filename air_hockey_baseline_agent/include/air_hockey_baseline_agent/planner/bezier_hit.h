/*
 * MIT License
 * Copyright (c) 2020-2021 Puze Liu, Davide Tateo
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

#ifndef SRC_BEZIER_HIT_H
#define SRC_BEZIER_HIT_H

#include <Eigen/Core>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"


namespace air_hockey_baseline_agent {
    class BezierHit {
    public:
        BezierHit(Eigen::Vector2d bound_lower, Eigen::Vector2d bound_upper, double rate, double height);

        bool plan(const Eigen::Vector2d &xStart, const Eigen::Vector2d &xHit, const Eigen::Vector2d &vHit,
                  trajectory_msgs::MultiDOFJointTrajectory &cartTraj);

        inline double getHitTime() const { return tHit_; };

        inline double getStopTime() const { return tStop_; };

    private:
        bool fit(const Eigen::Vector2d &xStart, const Eigen::Vector2d &xHit, const Eigen::Vector2d &vHit);

        bool getPoint(double t);

    private:
        Eigen::Vector2d boundLower_;
        Eigen::Vector2d boundUpper_;
        Eigen::Vector2d xStart_, xMiddle_, xHit_, vHit_, vHitNormalized_;    //Start point, middle point, final point, final velocity
        double stepSize_;
        double height_;
        double tHit_, tStop_;
        double aMax_;
        double z_, dz_dt_, dz_ddt_;        //Phase position, velocity, acceleration
        Eigen::Vector2d dx_dz_, dx_ddz_;       //Catesian velocity and acceleration w.r.t phase
        Eigen::Vector2d x_, dx_dt_, dx_ddt_;    //Catesian velocity and acceleration w.r.t time
        Eigen::Matrix<double, 5, 1> phaseCoeff_;

        trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;
    };
}

#endif //SRC_BEZIER_HIT_H
