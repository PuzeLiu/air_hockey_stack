#include "planner/bezierHit.h"

using namespace tactical_agent;

BezierHit::BezierHit(Vector2d bound_lower, Vector2d bound_upper, double rate, double height) {
    boundLower_ = bound_lower;
    boundUpper_ = bound_upper;
    rate_ = rate;
    height_ = height;

    viaPoint_.transforms.resize(1);
    viaPoint_.velocities.resize(1);
    viaPoint_.accelerations.resize(1);
}

bool BezierHit::plan(const Vector2d &xStart, const Vector2d &xHit, const Vector2d &vHit,
                     trajectory_msgs::MultiDOFJointTrajectory &cartTraj) {
    if(!fit(xStart, xHit, vHit)){
        return false;
    }
    double t = 0.;

    while (t <= tStop_){
        t += 1 / rate_;
        if(getPoint(t)) {
            cartTraj.points.push_back(viaPoint_);
        }
    }
    return true;
}

bool BezierHit::fit(const Vector2d& x0, const Vector2d& xf, const Vector2d& vf) {
    // Check both point are inside boundary
    for (int i = 0; i < 2; ++i) {
        if (x0[i] < boundLower_[i] or x0[i] > boundUpper_[i] or
            xf[i] < boundLower_[i] or xf[i] > boundUpper_[i] or
            vf.norm() == 0.) {
            ROS_INFO_STREAM(x0[i] << " " << xf[i] << " "<< boundLower_[i] << " " << boundUpper_[i]);
            cout << "Planner Failed: points out of boundary!" << endl;
            return false;
        }
    }

    xStart_ = x0;
    xHit_ = xf;
    vHit_ = vf;
    // Find Middle Point in hitting
    double alpha_min_hit = INFINITY;        // cross point at hitting phase
    for (int i = 0; i < 2; ++i) {
        if (vHit_[i] > 0) {
            alpha_min_hit = min(alpha_min_hit, (xHit_[i] - boundLower_[i]) / vHit_[i]);
        } else if (vf[i] < 0) {
            alpha_min_hit = min(alpha_min_hit, (xHit_[i] - boundUpper_[i]) / vHit_[i]);
        }
    }
    xMiddle_ = xHit_ - alpha_min_hit * vHit_;
    xMiddleStop_ = xHit_ + alpha_min_hit * vHit_;

    // Construct Quartic Polynomial
    double dz_dtf;
    if (vHit_[0] != 0.){
        dz_dtf = vHit_[0] / (xHit_[0] - xMiddle_[0]) / 2;
    } else {
        dz_dtf = vHit_[1] / (xHit_[1] - xMiddle_[1]) / 2;
    }

    tHit_ = 2 / dz_dtf;
    tStop_ = 3 / dz_dtf;
    phaseCoeff_[0] = 0.;
    phaseCoeff_[1] = 0.;
    phaseCoeff_[2] = 0.;
    phaseCoeff_[3] = pow(dz_dtf, 3) / 4.;
    phaseCoeff_[4] = -pow(dz_dtf, 4) / 16.;
    return true;
}

bool BezierHit::getPoint(double t) {
    if (t <= tHit_) {
        z = phaseCoeff_[3] * pow(t, 3) + phaseCoeff_[4] * pow(t, 4);
        dz_dt_ = 3 * phaseCoeff_[3] * pow(t, 2) + 4 * phaseCoeff_[4] * pow(t, 3);
        dz_ddt_ = 6 * phaseCoeff_[3] * t + 12 * phaseCoeff_[4] * pow(t, 2);

        x_ = xMiddle_ + pow((1 - z), 2) * (xStart_ - xMiddle_) + pow(z, 2) * (xHit_ - xMiddle_);
        dx_dz_ = (2 * (1 - z) * (xMiddle_ - xStart_) + 2 * z * (xHit_ - xMiddle_));
        dx_dt_ = dx_dz_ * dz_dt_;
        dx_ddz_ = 2 * (xHit_ - 2 * xMiddle_ + xStart_);
        dx_ddt_ = dx_ddz_ * pow(dz_dt_, 2) + dx_dz_ * dz_ddt_;
    }
    else if (t <= tStop_){
        z = phaseCoeff_[3] * pow(t, 3) + phaseCoeff_[4] * pow(t, 4) - 1;
        dz_dt_ = 3 * phaseCoeff_[3] * pow(t, 2) + 4 * phaseCoeff_[4] * pow(t, 3);
        dz_ddt_ = 6 * phaseCoeff_[3] * t + 12 * phaseCoeff_[4] * pow(t, 2);

        x_ = xMiddleStop_ + pow((1 - z), 2) * (xHit_ - xMiddleStop_) + pow(z, 2) * (xStart_ - xMiddleStop_) ;
        dx_dz_ = (2 * (1 - z) * (xMiddleStop_ - xHit_) + 2 * z * (xStart_ - xMiddleStop_));
        dx_dt_ = dx_dz_ * dz_dt_;
        dx_ddz_ = 2 * (xHit_ - 2 * xMiddleStop_ + xStart_);
        dx_ddt_ = dx_ddz_ * pow(dz_dt_, 2) + dx_dz_ * dz_ddt_;
    }
    else{
        return false;
    }
    viaPoint_.transforms[0].translation.x = x_[0];
    viaPoint_.transforms[0].translation.y = x_[1];
    viaPoint_.transforms[0].translation.z = height_;
    viaPoint_.velocities[0].linear.x = dx_dt_[0];
    viaPoint_.velocities[0].linear.y = dx_dt_[1];
    viaPoint_.velocities[0].linear.z = 0.0;
    viaPoint_.accelerations[0].linear.x = dx_ddt_[0];
    viaPoint_.accelerations[0].linear.y = dx_ddt_[1];
    viaPoint_.accelerations[0].linear.z = 0.0;
    viaPoint_.time_from_start = ros::Duration(t);
    return true;
}
