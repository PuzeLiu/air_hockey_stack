//
// Created by puze on 08.01.21.
//

#ifndef SRC_CUBIC_LINEAR_MOTION_H
#define SRC_CUBIC_LINEAR_MOTION_H

#include <Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace Eigen;

namespace AirHockey{
class CubicLinearMotion {
public:
    CubicLinearMotion(double rate, double height);
    ~CubicLinearMotion();
    bool plan(Vector2d pStart, Vector2d vStart, Vector2d pStop, Vector2d vStop, double tStop, trajectory_msgs::MultiDOFJointTrajectory& cartTraj);

private:
    double stepSize_;
    double height_;

    Vector2d xTmp_, vTmp_;
    trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;

    Matrix<double, 2, 4> coefficients_;

};
}


#endif //SRC_CUBIC_LINEAR_MOTION_H
