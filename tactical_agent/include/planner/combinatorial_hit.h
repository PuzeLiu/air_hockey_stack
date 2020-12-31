#ifndef SRC_COMBINATORIAL_HIT_H
#define SRC_COMBINATORIAL_HIT_H

#include <Eigen/Core>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

using namespace Eigen;
using namespace std;

namespace tactical_agent{
class CombinatorialHit {
public:
    CombinatorialHit(Vector2d bound_lower, Vector2d bound_upper, double rate, double height);
    ~CombinatorialHit();

    bool plan(const Vector2d &xStart, const Vector2d &xHit, const Vector2d &vHit,
              trajectory_msgs::MultiDOFJointTrajectory &cartTraj);

private:
    bool getMiddlePoint();
    bool getArcCenter();
    void fitPhase();
    bool getPoint(const double t);

private:
    Vector2d boundLower_;
    Vector2d boundUpper_;

    Vector2d xStart_, xMiddle_, xHit_;             //! Start point, middle point, final point, final velocity
    Vector2d vecDir1_, vecDir2_;                   //! Unit vector of linear 1 and 2
    Vector2d xArcCenter_, xVia1_, xVia2_;          //! Via point of arc
    double rate_;
    double height_;
    double vHitMag_;                               //! Magnitude of hitting velocity
    double tHit_, tStop_;
    double aMax_;                                  //! Maximum acceleration for stop movement
    double arcAngle_, arcRadius_, arcLength_;
    double clockWise_;                             //! Indicator of arc direction 1: clockwise, -1: counterclockwise
    double l1_, l2_, lHit_;                        //! Length of different segments
    double z_, dz_dt_, dz_ddt_;                    //! Phase variable (arc length, velocity, acceleration)
    Vector2d x_, dx_dt_, dx_ddt_;                  //! 2d position variable (arc length, velocity, acceleration)

    Matrix<double, 5, 1> phaseCoeff_;
    Matrix<double, 5, 1> stopPhaseCoeff_;
    trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint_;
};
}


#endif //SRC_COMBINATORIAL_HIT_H
