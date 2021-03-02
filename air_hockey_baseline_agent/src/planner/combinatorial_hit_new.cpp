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
	tDecelerateAngular = stepSize;
}

CombinatorialHitNew::~CombinatorialHitNew() = default;

bool CombinatorialHitNew::plan(const Eigen::Vector2d &x_start, const Eigen::Vector2d &v_start,
                               const Eigen::Vector2d &x_end, const Eigen::Vector2d &v_end,
                               trajectory_msgs::MultiDOFJointTrajectory &cartTraj) {
	xStart = x_start;
	xEnd = x_end;

	//! check the velocity at start and end point
	if ((v_start.norm() > 1e-3 && v_end.norm() > 1e-3) ||
	    (v_start.norm() < 1e-3 && v_end.norm() < 1e-3)) {
		cerr << "One of the velocity at start / end point should be zero" << endl;
		cerr << "Start: " << v_start.transpose() << endl;
		cerr << "End: " << v_end.transpose() << endl;
		return false;
	}
	if (v_start.norm() > 1e-3) {
		isStartPart = false;
		vDir = v_start.normalized();
		vMag = v_start.norm();
	} else {
		isStartPart = true;
		vDir = v_end.normalized();
		vMag = v_end.norm();
	}

	//! check the position at start and end point
	if ((xEnd - xStart).x() * vDir.x() < 0) {
		cerr << "Unable to plan the trajectory. The points relative position is not aligned with hitting direction" << endl;
		return false;
	}

	if (!getMiddlePoint()) {
		return false;
	}
	if (!getArcCenter()) {
		return false;
	}

	fitPhase();

	double t_prev;
	if (cartTraj.points.empty()) {
		t_prev = 0.;
	} else {
		t_prev = cartTraj.points.back().time_from_start.toSec();
	}

	double t = 0.;

	while (t + stepSize <= tEnd + tDecelerateAngular) {
		t += stepSize;
		getPoint(t);
		viaPoint.time_from_start = ros::Duration(t + t_prev);
		cartTraj.points.push_back(viaPoint);
	}

	cartTraj.header.stamp = ros::Time::now();
	return true;
}

bool CombinatorialHitNew::getMiddlePoint() {
	for (int i = 0; i < 2; ++i) {
		xStart[i] = boost::algorithm::clamp(xStart[i], boundLower[i], boundUpper[i]);
		xEnd[i] = boost::algorithm::clamp(xEnd[i], boundLower[i], boundUpper[i]);
	}

	//! update the lower bound and upper bound of x direction to start position
	Vector2d boundLowerTmp = boundLower;
	Vector2d boundUpperTmp = boundUpper;

	boundLowerTmp[0] = max(min(xStart[0], xEnd[0]), boundLowerTmp[0]);
	boundUpperTmp[0] = min(max(xStart[0], xEnd[0]), boundUpperTmp[0]);

	// Find Middle Point in hitting
	double alphaMiddle = DBL_MAX;        // cross point at hitting phase
	if (isStartPart) {
		for (int i = 0; i < 2; ++i) {
			if (vDir[i] > 0) {
				alphaMiddle = min(alphaMiddle, (xEnd[i] - boundLowerTmp[i]) / vDir[i]);
			} else if (vDir[i] < 0) {
				alphaMiddle = min(alphaMiddle, (xEnd[i] - boundUpperTmp[i]) / vDir[i]);
			}
		}
		xMiddle = xEnd - alphaMiddle * vDir;
	} else {
		for (int i = 0; i < 2; ++i) {
			if (vDir[i] > 0) {
				alphaMiddle = min(alphaMiddle, (boundUpperTmp[i] - xStart[i]) / vDir[i]);
			} else if (vDir[i] < 0) {
				alphaMiddle = min(alphaMiddle, (boundLowerTmp[i] - xStart[i]) / vDir[i]);
			}
		}
		xMiddle = xStart + alphaMiddle * vDir;
	}

	vecDir1 = (xMiddle - xStart).normalized();
	vecDir2 = (xEnd - xMiddle).normalized();
	return true;
}

bool CombinatorialHitNew::getArcCenter() {
	double t_min = min((xStart - xMiddle).norm(), (xMiddle - xEnd).norm());

	xVia1 = xMiddle - t_min * vecDir1;
	xVia2 = xMiddle + t_min * vecDir2;

	l1 = (xVia1 - xStart).norm();
	l2 = (xEnd - xVia2).norm();

	Vector2d vecNorm1(-vecDir1.y(), vecDir1.x());
	Vector2d vecNorm2(-vecDir2.y(), vecDir2.x());

	//! find arc radius r1, r2, solve linear equations: x1 + r1 * n1 = x2 + r2 * n2
	Matrix2d A(2, 2);
	Vector2d b;
	A << vecNorm1.x(), -vecNorm2.x(), vecNorm1.y(), -vecNorm2.y();
	b = xVia2 - xVia1;
	Vector2d r1 = A.householderQr().solve(b);
	arcRadius = r1.x();

	if (r1.hasNaN()) {
		arcRadius = 0;
	}
	xArcCenter = xVia1 + arcRadius * vecNorm1;


	//! find arcAngle, positive if counterclockwise
	arcAngle = acos(vecNorm1.dot(vecNorm2));

	counterClockWise = 1;
	if (vecNorm1.x() * vecNorm2.y() - vecNorm1.y() * vecNorm2.x() < 0) {
		counterClockWise = -1;
	}

	arcLength = abs(arcAngle * arcRadius);

	lHit = l1 + arcLength + l2;
	return true;
}

void CombinatorialHitNew::fitPhase() {
	if (isStartPart) {
		tEnd = 2 * lHit / vMag;
		phaseCoeff[0] = 0.;
		phaseCoeff[1] = 0.;
		phaseCoeff[2] = 0.;
		phaseCoeff[3] = pow(vMag, 3) / 4 / pow(l1 + arcLength + l2, 2);
		phaseCoeff[4] = -pow(vMag, 4) / 16 / pow(l1 + arcLength + l2, 3);
	} else {
		tEnd = 2 * lHit / vMag;
		phaseCoeff[0] = 0.;
		phaseCoeff[1] = vMag;
		phaseCoeff[2] = 0.;
		phaseCoeff[3] = -vMag / pow(tEnd, 2);
		phaseCoeff[4] = - phaseCoeff[3] / 2 / tEnd;
	}

}

void CombinatorialHitNew::getPoint(const double t) {
	if (t <= tEnd) {
		z = phaseCoeff[1] * t + phaseCoeff[3] * pow(t, 3) + phaseCoeff[4] * pow(t, 4);
		dz_dt = phaseCoeff[1] + 3 * phaseCoeff[3] * pow(t, 2) + 4 * phaseCoeff[4] * pow(t, 3);
        dz_ddt = 6 * phaseCoeff[3] * t + 12 * phaseCoeff[4] * pow(t, 2);

		if (z <= l1) {
			x = xStart + z * vecDir1;
			dx_dt = dz_dt * vecDir1;
            dx_ddt = dz_ddt * vecDir1;
			aAngularAcc.setZero();
		} else if (z <= l1 + arcLength) {
			double angleCur = (z - l1) / abs(arcRadius) * counterClockWise;
			Rotation2Dd rot(angleCur);
			x = rot.matrix() * (xVia1 - xArcCenter) + xArcCenter;
			dx_dt = rot.matrix() * vecDir1 * dz_dt;
			aAngularAcc = (xArcCenter - x) * pow(dx_dt.norm(), 2) / abs(arcRadius);
			dx_ddt = dz_ddt * dx_dt.normalized() + aAngularAcc;
		} else if (z <= l1 + arcLength + l2) {
			x = xVia2 + (z - l1 - arcLength) * vecDir2;
			dx_dt = dz_dt * vecDir2;
			aAngularAcc.setZero();
			dx_ddt = dz_ddt * vecDir2;
		}
	} else if (t <= tEnd + tDecelerateAngular){
		auto psi = (t - tEnd) / tDecelerateAngular;
		dx_dt += (1 - psi) * aAngularAcc * stepSize;
		x += dx_dt * stepSize;
	} else {
		x += dx_dt * stepSize;
	}
	viaPoint.transforms[0].translation.x = x[0];
	viaPoint.transforms[0].translation.y = x[1];
	viaPoint.transforms[0].translation.z = height;
	viaPoint.velocities[0].linear.x = dx_dt[0];
	viaPoint.velocities[0].linear.y = dx_dt[1];
	viaPoint.velocities[0].linear.z = 0.0;
}
