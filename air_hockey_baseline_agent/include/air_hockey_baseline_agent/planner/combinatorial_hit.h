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


#ifndef SRC_COMBINATORIAL_HIT_H
#define SRC_COMBINATORIAL_HIT_H

#include <Eigen/Core>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

namespace air_hockey_baseline_agent{
class CombinatorialHit {
public:
    CombinatorialHit(Eigen::Vector2d bound_lower, Eigen::Vector2d bound_upper, double rate, double height);
    ~CombinatorialHit();

    bool plan(const Eigen::Vector2d &xStart, const Eigen::Vector2d &xHit, const Eigen::Vector2d &vHit,
              trajectory_msgs::MultiDOFJointTrajectory &cartTraj, double stopTime=0.2);

private:
    bool getMiddlePoint();
    bool getArcCenter();
    void fitPhase(double stopTime=0.2);
    void getPoint(const double t);

private:
    Eigen::Vector2d boundLower_;
    Eigen::Vector2d boundUpper_;

    Eigen::Vector2d xStart_, xMiddle_, xHit_;      //! Start point, middle point, final point, final velocity
    Eigen::Vector2d vecDir1_, vecDir2_;            //! Unit vector of linear 1 and 2
    Eigen::Vector2d xArcCenter_, xVia1_, xVia2_;   //! Via point of arc
    double stepSize_;
    double height_;
    double vHitMag_;                               //! Magnitude of hitting velocity
    double tHit_, tStop_;
    double aMax_;                                  //! Maximum acceleration for stop movement
    double arcAngle_, arcRadius_, arcLength_;
    double clockWise_;                             //! Indicator of arc direction 1: clockwise, -1: counterclockwise
    double l1_, l2_, lHit_;                        //! Length of different segments
    double z_, dz_dt_, dz_ddt_;                    //! Phase variable (arc length, velocity, acceleration)
    Eigen::Vector2d x_, dx_dt_, dx_ddt_;           //! 2d position variable (arc length, velocity, acceleration)

    Eigen::Matrix<double, 5, 1> phaseCoeff_;
    Eigen::Matrix<double, 5, 1> stopPhaseCoeff_;
    trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;
};
}


#endif //SRC_COMBINATORIAL_HIT_H
