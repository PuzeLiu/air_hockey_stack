//
// Created by puze on 08.12.20.
//

#ifndef SRC_BEZIER_HIT_H
#define SRC_BEZIER_HIT_H

#include <Eigen/Core>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

using namespace std;
using namespace Eigen;

namespace tactical_agent {
    class BezierHit {
    public:
        BezierHit(Vector2d bound_lower, Vector2d bound_upper, double rate, double height);

        bool plan(const Vector2d &xStart, const Vector2d &xHit, const Vector2d &vHit,
                  trajectory_msgs::MultiDOFJointTrajectory &cartTraj);

        inline double getHitTime() const { return tHit_; };

        inline double getStopTime() const { return tStop_; };

    private:
        bool fit(const Vector2d &xStart, const Vector2d &xHit, const Vector2d &vHit);

        bool getPoint(double t);

    private:
        Vector2d boundLower_;
        Vector2d boundUpper_;
        Vector2d xStart_, xMiddle_, xHit_, vHit_, vHitNormalized_;    //Start point, middle point, final point, final velocity
        double stepSize_;
        double height_;
        double tHit_, tStop_;
        double aMax_;
        double z_, dz_dt_, dz_ddt_;        //Phase position, velocity, acceleration
        Vector2d dx_dz_, dx_ddz_;       //Catesian velocity and acceleration w.r.t phase
        Vector2d x_, dx_dt_, dx_ddt_;    //Catesian velocity and acceleration w.r.t time
        Matrix<double, 5, 1> phaseCoeff_;

        trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;
    };
}

#endif //SRC_BEZIER_HIT_H
