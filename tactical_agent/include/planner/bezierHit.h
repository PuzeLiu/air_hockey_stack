//
// Created by puze on 08.12.20.
//

#ifndef SRC_BEZIERHIT_H
#define SRC_BEZIERHIT_H

#include "ros/ros.h"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class BezierHit{
public:
    BezierHit(Vector2d bound_lower, Vector2d bound_upper, double height=0.);

//    bool fit(const Vector2d& xStart, const Vector2d& xHit, const Vector2d& vHit);
//
//    bool getPoint(double t, null_space_optimization::CartersianTrajectory& msg);
//
//    vector<null_space_optimization::CartersianTrajectory> getTrajectory(double stepSize);
//
//    inline double getHitTime() const{return tHit_;};
//
//    inline double getStopTime() const{return tStop_;};

public:
    Vector2d boundLower_;
    Vector2d boundUpper_;
private:

    Vector2d xStart_, xMiddle_, xHit_, vHit_;    //Start point, middle point, final point, final velocity
    double height_;
    double tHit_, tStop_;
    double z, dz_dt, dz_ddt;        //Phase position, velocity, acceleration
    Vector2d dx_dz_, dx_ddz_;       //Catesian velocity and acceleration w.r.t phase
    Vector2d x_, dx_dt_, dx_ddt;    //Catesian velocity and acceleration w.r.t time
    Matrix<double, 5, 1> phaseCoeff_;

    Vector2d xMiddleStop_;
};


#endif //SRC_BEZIERHIT_H
