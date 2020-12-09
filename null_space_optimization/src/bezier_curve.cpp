//
// Created by puze on 08.12.20.
//

#include "bezier_curve.h"

BezierCurve2D::BezierCurve2D(Vector2d bound_lower, Vector2d bound_upper, double height) {
    boundLower_ = bound_lower;
    boundUpper_ = bound_upper;
    height_ = height;
}

bool BezierCurve2D::fit(const Vector2d& x0, const Vector2d& xf, const Vector2d& vf) {
    // Check both point are inside boundary
    for (int i = 0; i < 2; ++i) {
        if (x0[i] < boundLower_[i] or x0[i] > boundUpper_[i] or
            xf[i] < boundLower_[i] or xf[i] > boundUpper_[i] or
            vf.norm() == 0.) {
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
    double dz_dtf = vHit_[0] / (xHit_[0] - xMiddle_[0]) / 2;
    tHit_ = 2 / dz_dtf;
    tStop_ = 3 / dz_dtf;
    phaseCoeff_[0] = 0.;
    phaseCoeff_[1] = 0.;
    phaseCoeff_[2] = 0.;
    phaseCoeff_[3] = pow(dz_dtf, 3) / 4.;
    phaseCoeff_[4] = -pow(dz_dtf, 4) / 16.;

    return true;
}

void BezierCurve2D::getPoint(double t, null_space_optimization::CartersianTrajectory& msg) {
    if (t <= tHit_) {
        z = phaseCoeff_[3] * pow(t, 3) + phaseCoeff_[4] * pow(t, 4);
        dz_dt = 3 * phaseCoeff_[3] * pow(t, 2) + 4 * phaseCoeff_[4] * pow(t, 3);
        dz_ddt = 6 * phaseCoeff_[3] * t + 12 * phaseCoeff_[4] * pow(t, 2);

        x_ = xMiddle_ + pow((1 - z), 2) * (xStart_ - xMiddle_) + pow(z, 2) * (xHit_ - xMiddle_);
        dx_dz_ = (2 * (1 - z) * (xMiddle_ - xStart_) + 2 * z * (xHit_ - xMiddle_));
        dx_dt_ = dx_dz_ * dz_dt;
        dx_ddz_ = 2 * (xHit_ - 2 * xMiddle_ + xStart_);
        dx_ddt = dx_ddz_ * pow(dz_dt, 2) + dx_dz_ * dz_ddt;
    }
    else if (t <= tStop_){
        z = phaseCoeff_[3] * pow(t, 3) + phaseCoeff_[4] * pow(t, 4) - 1;
        dz_dt = 3 * phaseCoeff_[3] * pow(t, 2) + 4 * phaseCoeff_[4] * pow(t, 3);
        dz_ddt = 6 * phaseCoeff_[3] * t + 12 * phaseCoeff_[4] * pow(t, 2);

        x_ = xMiddleStop_ + pow((1 - z), 2) * (xHit_ - xMiddleStop_) + pow(z, 2) * (xStart_ - xMiddleStop_) ;
        dx_dz_ = (2 * (1 - z) * (xMiddleStop_ - xHit_) + 2 * z * (xStart_ - xMiddleStop_));
        dx_dt_ = dx_dz_ * dz_dt;
        dx_ddz_ = 2 * (xHit_ - 2 * xMiddleStop_ + xStart_);
        dx_ddt = dx_ddz_ * pow(dz_dt, 2) + dx_dz_ * dz_ddt;
    }
    else{
        return;
    }
    msg.position.x = x_[0];
    msg.position.y = x_[1];
    msg.position.z = height_;
    msg.velocity.x = dx_dt_[0];
    msg.velocity.y = dx_dt_[1];
    msg.velocity.z = 0.;
}
