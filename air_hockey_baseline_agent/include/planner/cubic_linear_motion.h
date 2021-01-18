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
    bool plan(const Vector2d& pStart,
              const Vector2d& vStart,
              const Vector2d& pStop,
              const Vector2d& vStop,
              double tStop,
              trajectory_msgs::MultiDOFJointTrajectory& cartTraj);

    bool plan(const Vector3d& pStart,
              const Vector3d& vStart,
              const Vector3d& pStop,
              const Vector3d& vStop,
              double tStop,
              trajectory_msgs::MultiDOFJointTrajectory& cartTraj);

private:
    double stepSize_;
    double height_;


    trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;



};
}


#endif //SRC_CUBIC_LINEAR_MOTION_H
