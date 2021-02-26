/*
 * MIT License
 * Copyright (c) 2021 Puze Liu, Davide Tateo
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

#include "air_hockey_baseline_agent/planner/combinatorial_hit_new.h"
#include <boost/algorithm/clamp.hpp>

using namespace std;
using namespace Eigen;
using namespace air_hockey_baseline_agent;

CombinatorialHitNew::CombinatorialHitNew(Eigen::Vector2d bound_lower, Eigen::Vector2d bound_upper, double rate,
                                         double table_height) {
    boundLower = bound_lower;
    boundUpper = bound_upper;
    stepSize = 1 / rate;
    height = table_height;

    viaPoint.transforms.resize(1);
    viaPoint.velocities.resize(1);
}

CombinatorialHitNew::~CombinatorialHitNew() = default;

bool CombinatorialHitNew::plan(const Eigen::Vector2d &x_start, const Eigen::Vector2d &x_hit,
                               const Eigen::Vector2d &v_hit,
                               trajectory_msgs::MultiDOFJointTrajectory &cartTraj) {
    xStart = x_start;
    xHit = x_hit;
    vecDir2 = v_hit.normalized();
    vHitMag = v_hit.norm();

    if (!getMiddlePoint()) {
        return false;
    }
    if (!getArcCenter()) {
        return false;
    }

    fitPhase();

    double t_prev;
    if (cartTraj.points.size() == 0) {
        t_prev = 0.;
    } else {
        t_prev = cartTraj.points.back().time_from_start.toSec();
    }

    double t = 0.;

    while (t <= tHitPhase + tStopPhase) {
        t += stepSize;
        getPoint(t);
        viaPoint.time_from_start = ros::Duration(t + t_prev);
        cartTraj.points.push_back(viaPoint);
    }

    //! Append one additional point
    t += stepSize;
    getPoint(t);
    viaPoint.time_from_start = ros::Duration(t + t_prev);
    cartTraj.points.push_back(viaPoint);
    cartTraj.header.stamp = ros::Time::now();
    return true;
}

bool CombinatorialHitNew::getMiddlePoint() {
    if (vHitMag < 1e-3) {
        cout << "Hit velocity should not less than: 1e-3" << endl;
        return false;
    }

    for (int i = 0; i < 2; ++i) {
        xStart[i] = boost::algorithm::clamp(xStart[i], boundLower[i], boundUpper[i]);
        xHit[i] = boost::algorithm::clamp(xHit[i], boundLower[i], boundUpper[i]);
    }

    //! update the lower bound and upper bound of x direction to start position
    Vector2d boundLowerTmp = boundLower;
    Vector2d boundUpperTmp = boundUpper;
    boundLowerTmp[0] = max(xStart[0], boundLowerTmp[0]);
    boundUpperTmp[0] = min(xHit[0] + 0.05, boundUpperTmp[0]);
    xStop << boundUpperTmp[0], 0.;

    // Find Middle Point in hitting
    double alphaMiddle1 = DBL_MAX;        // cross point at hitting phase
    double alphaMiddle2 = DBL_MAX;       // cross point at hitting phase
    for (int i = 0; i < 2; ++i) {
        if (vecDir2[i] > 0) {
            alphaMiddle1 = min(alphaMiddle1, (xHit[i] - boundLowerTmp[i]) / vecDir2[i]);
            alphaMiddle2 = min(alphaMiddle2, (boundUpperTmp[i] - xHit[i]) / vecDir2[i]);
        } else if (vecDir2[i] < 0) {
            alphaMiddle1 = min(alphaMiddle1, (xHit[i] - boundUpperTmp[i]) / vecDir2[i]);
            alphaMiddle2 = min(alphaMiddle2, (boundLowerTmp[i] - xHit[i]) / vecDir2[i]);
        }
    }
    xMiddle1 = xHit - alphaMiddle1 * vecDir2;
    xMiddle2 = xHit + alphaMiddle2 * vecDir2;
    vecDir1 = (xMiddle1 - xStart).normalized();
    vecDir3 = (xStop - xMiddle2).normalized();
    return true;
}

bool CombinatorialHitNew::getArcCenter() {
    double t_min_1 = min((xStart - xMiddle1).norm(), (xMiddle1 - xHit).norm());
    double t_min_2 = min((xStop - xMiddle2).norm(), (xMiddle2 - xHit).norm());

    xVia1 = xMiddle1 - t_min_1 * vecDir1;
    xVia2 = xMiddle1 + t_min_1 * vecDir2;
    xVia3 = xMiddle2 - t_min_2 * vecDir2;
    xVia4 = xMiddle2 + t_min_2 * vecDir3;

    l1 = (xVia1 - xStart).norm();
    l2 = (xHit - xVia2).norm();
    l3 = (xVia3 - xHit).norm();
    l4 = (xStop - xVia4).norm();

    Vector2d vecNorm1(-vecDir1.y(), vecDir1.x());
    Vector2d vecNorm2(-vecDir2.y(), vecDir2.x());
    Vector2d vecNorm3(-vecDir3.y(), vecDir3.x());

    //! find arc radius r1, r2, solve linear equations: x1 + r1 * n1 = x2 + r2 * n2
    Matrix2d A1(2, 2), A2(2, 2);
    Vector2d b1, b2;
    A1 << vecNorm1.x(), -vecNorm2.x(), vecNorm1.y(), -vecNorm2.y();
    b1 = xVia2 - xVia1;
    auto r1 = A1.inverse() * b1;
    arcRadius1 = r1.x();
    // second arc
    A2 << vecNorm2.x(), -vecNorm3.x(), vecNorm2.y(), -vecNorm3.y();
    b2 = xVia4 - xVia3;
    auto r2 = A2.inverse() * b2;
    arcRadius2 = r2.x();

    if (r1.hasNaN()){
        arcRadius1 = 0;
    }
    if (r2.hasNaN()){
        arcRadius2 = 0;
    }
    xArcCenter1 = xVia1 + arcRadius1 * vecNorm1;
    xArcCenter2 = xVia3 + arcRadius2 * vecNorm2;

    //! find arcAngle, positive if counterclockwise
    arcAngle1 = acos(vecNorm1.dot(vecNorm2));
    arcAngle2 = acos(vecNorm2.dot(vecNorm3));

    counterClockWise1 = 1;
    counterClockWise2 = 1;
    if (vecNorm1.x() * vecNorm2.y() - vecNorm1.y() * vecNorm2.x() < 0) {
        counterClockWise1 = -1;
    }
    if (vecNorm2.x() * vecNorm3.y() - vecNorm2.y() * vecNorm3.x() < 0) {
        counterClockWise2 = -1;
    }

    arcLength1 = abs(arcAngle1 * arcRadius1);
    arcLength2 = abs(arcAngle2 * arcRadius2);

    lHit = l1 + arcLength1 + l2 + l3 + arcLength2 + l4;
    return true;
}

void CombinatorialHitNew::fitPhase() {
    tHitPhase = 2 * (l1 + arcLength1 + l2) / vHitMag;
    phaseCoeff[0] = 0.;
    phaseCoeff[1] = 0.;
    phaseCoeff[2] = 0.;
    phaseCoeff[3] = pow(vHitMag, 3) / 4 / pow(l1 + arcLength1 + l2, 2);
    phaseCoeff[4] = -pow(vHitMag, 4) / 16 / pow(l1 + arcLength1 + l2, 3);

    tStopPhase = 2 * (l3 + arcLength2 + l4) / vHitMag;
    stopPhaseCoeff[0] = 0.;
    stopPhaseCoeff[1] = vHitMag;
    stopPhaseCoeff[2] = 0.;
    stopPhaseCoeff[3] = -vHitMag / pow(tStopPhase, 2);
    stopPhaseCoeff[4] = vHitMag / 2 / pow(tStopPhase, 3);

}

void CombinatorialHitNew::getPoint(const double t) {
    if (t <= tHitPhase) {
        z = phaseCoeff[3] * pow(t, 3) + phaseCoeff[4] * pow(t, 4);
        dz_dt = 3 * phaseCoeff[3] * pow(t, 2) + 4 * phaseCoeff[4] * pow(t, 3);
//        dz_ddt = 6 * phaseCoeff[3] * t + 12 * phaseCoeff[4] * pow(t, 2);

        if (z <= l1) {
            x = xStart + z * vecDir1;
            dx_dt = dz_dt * vecDir1;
//            dx_ddt = dz_ddt * vecDir1;
        } else if (z <= l1 + arcLength1) {
            double angleCur = (z - l1) / abs(arcRadius1) * counterClockWise1;
            Rotation2Dd rot(angleCur);
            x = rot.matrix() * (xVia1 - xArcCenter1) + xArcCenter1;
            dx_dt = rot.matrix() * vecDir1 * dz_dt;
        } else if (z <= l1 + arcLength1 + l2) {
            x = xVia2 + (z - l1 - arcLength1) * vecDir2;
            dx_dt = dz_dt * vecDir2;
//            dx_ddt = dz_ddt * vecDir2;
        }

    } else if (t <= tHitPhase + tStopPhase) {
        z = stopPhaseCoeff[1] * (t - tHitPhase) + stopPhaseCoeff[3] * pow(t - tHitPhase, 3)
            + stopPhaseCoeff[4] * pow(t - tHitPhase, 4);
        dz_dt = stopPhaseCoeff[1] + 3 * stopPhaseCoeff[3] * pow(t - tHitPhase, 2)
                + 4 * stopPhaseCoeff[4] * pow(t - tHitPhase, 3);
//        dz_ddt = 6 * stopPhaseCoeff[3] * (t - tHitPhase) + 12 * stopPhaseCoeff[4] * pow(t - tHitPhase, 2);

        if (z <= l3) {
            x = xHit + z * vecDir2;
            dx_dt = dz_dt * vecDir2;
//            dx_ddt = dz_ddt * vecDir2;
        } else if (z <= l3 + arcLength2) {
            double angleCur = (z - l3) / abs(arcRadius2) * counterClockWise2;
            Rotation2Dd rot(angleCur);
            x = rot.matrix() * (xVia3 - xArcCenter2) + xArcCenter2;
            dx_dt = rot.matrix() * vecDir2 * dz_dt;
        } else if (z <= l3 + arcLength2 + l4) {
            x = xVia4 + (z - l3 - arcLength2) * vecDir3;
            dx_dt = dz_dt * vecDir3;
//            dx_ddt = dz_ddt * vecDir3;
        }
    } else {
        z = stopPhaseCoeff[1] * (tStopPhase) + stopPhaseCoeff[3] * pow(tStopPhase, 3)
            + stopPhaseCoeff[4] * pow(tStopPhase, 4);
        dz_dt = stopPhaseCoeff[1] + 3 * stopPhaseCoeff[3] * pow(tStopPhase, 2)
                + 4 * stopPhaseCoeff[4] * pow(tStopPhase, 3);
        x = xVia4 + (z - l3 - arcLength2) * vecDir3;
        dx_dt = dz_dt * vecDir3;
    }
    viaPoint.transforms[0].translation.x = x[0];
    viaPoint.transforms[0].translation.y = x[1];
    viaPoint.transforms[0].translation.z = height;
    viaPoint.velocities[0].linear.x = dx_dt[0];
    viaPoint.velocities[0].linear.y = dx_dt[1];
    viaPoint.velocities[0].linear.z = 0.0;
}
