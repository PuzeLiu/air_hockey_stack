//
// Created by puze on 08.12.20.
//

#ifndef SRC_BEZIER_CURVE_H
#define SRC_BEZIER_CURVE_H

#include "ros/ros.h"
#include <Eigen/Core>

#include "null_space_optimization/CartersianTrajectory.h"

using namespace std;
using namespace Eigen;
class BezierCurve2D{
public:
    BezierCurve2D(Vector2d bound_lower, Vector2d bound_upper, double height=0.);

    bool fit(const Vector2d& x0, const Vector2d& xf, const Vector2d& vf);

    void getPoint(double t, null_space_optimization::CartersianTrajectory& msg);

    inline double getHitTime(){return tHit_;};

    inline double getStopTime(){return tStop_;};
private:
    Vector2d boundLower_;
    Vector2d boundUpper_;
    Vector2d xStart_, xMiddle_, xHit_, vHit_;    //Start point, middle point, final point, final velocity
    double height_;
    double tHit_, tStop_;
    double z, dz_dt, dz_ddt;        //Phase position, velocity, acceleration
    Vector2d dx_dz_, dx_ddz_;       //Catesian velocity and acceleration w.r.t phase
    Vector2d x_, dx_dt_, dx_ddt;    //Catesian velocity and acceleration w.r.t time
    Matrix<double, 5, 1> phaseCoeff_;

    Vector2d xMiddleStop_;
};


#endif //SRC_BEZIER_CURVE_H
