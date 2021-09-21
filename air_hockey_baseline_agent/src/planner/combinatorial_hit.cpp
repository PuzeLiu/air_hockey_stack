/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
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

#include "air_hockey_baseline_agent/planner/combinatorial_hit.h"

#include <Eigen/Geometry>
#include <boost/algorithm/clamp.hpp>

using namespace air_hockey_baseline_agent;
using namespace Eigen;
using namespace std;

CombinatorialHit::CombinatorialHit(Vector2d bound_lower, Vector2d bound_upper,
		double rate, double height) {
	boundLower_ = bound_lower;
	boundUpper_ = bound_upper;
	stepSize_ = 1 / rate;
	height_ = height;

	viaPoint_.transforms.resize(1);
	viaPoint_.velocities.resize(1);
}

CombinatorialHit::~CombinatorialHit() {

}

bool CombinatorialHit::plan(const Vector2d &xStart, const Vector2d &xHit,
		const Vector2d &vHit,
		trajectory_msgs::MultiDOFJointTrajectory &cartTraj, double stopTime) {
	xStart_ = xStart;
	xHit_ = xHit;
	vecDir2_ = vHit.normalized();
	vHitMag_ = vHit.norm();

	if (!getMiddlePoint()) {
		return false;
	}
	if (!getArcCenter()) {
		return false;
	}
	fitPhase(stopTime);

	double t_prev;
	if (cartTraj.points.size() == 0) {
		t_prev = 0.;
	} else {
		t_prev = cartTraj.points.back().time_from_start.toSec();
	}

	double t = 0.;

	while (t <= tStop_) {
		t += stepSize_;
		getPoint(t);
		viaPoint_.time_from_start = ros::Duration(t + t_prev);
		cartTraj.points.push_back(viaPoint_);
	}
	//! Append one additional point
	t += stepSize_;
	getPoint(t);
	viaPoint_.time_from_start = ros::Duration(t + t_prev);
	cartTraj.points.push_back(viaPoint_);
	cartTraj.header.stamp = ros::Time::now();
	return true;
}

bool CombinatorialHit::getMiddlePoint() {
	if (vHitMag_ < 1e-3) {
		cout << "Hit velocity should not less than: 1e-3" << endl;
	}

	for (int i = 0; i < 2; ++i) {
		xStart_[i] = boost::algorithm::clamp(xStart_[i], boundLower_[i],
				boundUpper_[i]);
		xHit_[i] = boost::algorithm::clamp(xHit_[i], boundLower_[i],
				boundUpper_[i]);
	}

	//! update the lower bound and upper bound of x direction to start position
	Vector2d boundLowerTmp = boundLower_;
	Vector2d boundUpperTmp = boundUpper_;
	boundLowerTmp[0] = min(xStart_[0], boundLower_[0]);
	boundUpperTmp[0] = max(xStart_[0], xHit_[0]);

	// Find Middle Point in hitting
	double alpha_min_hit = INFINITY;        // cross point at hitting phase
	for (int i = 0; i < 2; ++i) {
		if (vecDir2_[i] > 0) {
			alpha_min_hit = min(alpha_min_hit,
					(xHit_[i] - boundLowerTmp[i]) / vecDir2_[i]);
		} else if (vecDir2_[i] < 0) {
			alpha_min_hit = min(alpha_min_hit,
					(xHit_[i] - boundUpperTmp[i]) / vecDir2_[i]);
		}
	}
	xMiddle_ = xHit_ - alpha_min_hit * vecDir2_;
	vecDir1_ = (xMiddle_ - xStart_).normalized();
	return true;
}

bool CombinatorialHit::getArcCenter() {
	double t_min = min((xStart_ - xMiddle_).norm(), (xMiddle_ - xHit_).norm());
	xVia1_ = xMiddle_ - t_min * vecDir1_;
	xVia2_ = xMiddle_ + t_min * vecDir2_;

	l1_ = (xVia1_ - xStart_).norm();
	l2_ = (xHit_ - xVia2_).norm();

	Vector2d vecNorm1(-vecDir1_.y(), vecDir1_.x());
	Vector2d vecNorm2(-vecDir2_.y(), vecDir2_.x());

	if (vecDir2_[0] > 1e-3) {
		arcRadius_ = (xVia2_.x() - xVia1_.x()) / (vecNorm1.x() - vecNorm2.x());
	} else {
		arcRadius_ = (xVia2_.y() - xVia1_.y()) / (vecNorm1.y() - vecNorm2.y());
	}

	arcAngle_ = acos(vecNorm1.dot(vecNorm2));
	arcLength_ = arcAngle_ * abs(arcRadius_);
	xArcCenter_ = xVia1_ + vecNorm1 * arcRadius_;
	if (vecNorm1.x() * vecNorm2.y() - vecNorm1.y() * vecNorm2.x() >= 0) {
		clockWise_ = 1.;
	} else {
		clockWise_ = -1.;
	}

	if ((xArcCenter_ - (xVia2_ + vecNorm2 * arcRadius_)).norm() > 1e-6
			|| l1_ * l2_ > 1e-6) {
		cout << ((xArcCenter_ - (xVia2_ + vecNorm2 * arcRadius_)).norm() > 1e-6)
				<< endl;
		cout << l1_ << "  l2: " << l2_ << endl;
		cout << "Center is wrong" << endl;
		return false;
	}

	lHit_ = l1_ + l2_ + arcLength_;
	return true;
}

void CombinatorialHit::fitPhase(double stopTime) {
	tHit_ = 2 * lHit_ / vHitMag_;
	tStop_ = tHit_ + stopTime;
	phaseCoeff_[0] = 0.;
	phaseCoeff_[1] = 0.;
	phaseCoeff_[2] = 0.;
	phaseCoeff_[3] = pow(vHitMag_, 3) / 4 / pow(lHit_, 2);
	phaseCoeff_[4] = -pow(vHitMag_, 4) / 16 / pow(lHit_, 3);

	stopPhaseCoeff_[0] = 0.;
	stopPhaseCoeff_[1] = vHitMag_;
	stopPhaseCoeff_[2] = 0.;
	stopPhaseCoeff_[3] = -vHitMag_ / pow(tStop_ - tHit_, 2);
	stopPhaseCoeff_[4] = vHitMag_ / 2 / pow(tStop_ - tHit_, 3);
}

void CombinatorialHit::getPoint(const double t) {
	if (t <= tHit_) {
		z_ = phaseCoeff_[3] * pow(t, 3) + phaseCoeff_[4] * pow(t, 4);
		dz_dt_ = 3 * phaseCoeff_[3] * pow(t, 2)
				+ 4 * phaseCoeff_[4] * pow(t, 3);
//        dz_ddt_ = 6 * phaseCoeff_[3] * t + 12 * phaseCoeff_[4] * pow(t, 2);

		if (z_ <= l1_) {
			x_ = xStart_ + z_ * vecDir1_;
			dx_dt_ = dz_dt_ * vecDir1_;
//            dx_ddt_ = dz_ddt_ * vecDir1_;
		} else if (z_ <= l1_ + arcLength_) {
			double angleCur = (z_ - l1_) / abs(arcRadius_) * clockWise_;
			Rotation2Dd rot(angleCur);
			x_ = rot.matrix() * (xVia1_ - xArcCenter_) + xArcCenter_;
			dx_dt_ = rot.matrix() * vecDir1_ * dz_dt_;
		} else if (z_ <= lHit_) {
			x_ = xVia2_ + (z_ - l1_ - arcLength_) * vecDir2_;
			dx_dt_ = dz_dt_ * vecDir2_;
//            dx_ddt_ = dz_ddt_ * vecDir2_;
		}
	} else if (t <= tStop_) {
		z_ = stopPhaseCoeff_[1] * (t - tHit_)
				+ stopPhaseCoeff_[3] * pow(t - tHit_, 3)
				+ stopPhaseCoeff_[4] * pow(t - tHit_, 4);
		dz_dt_ = stopPhaseCoeff_[1] + 3 * stopPhaseCoeff_[3] * pow(t - tHit_, 2)
				+ 4 * stopPhaseCoeff_[4] * pow(t - tHit_, 3);
//        dz_ddt_ = 6 * stopPhaseCoeff_[3] * (t - tHit_) + 12 * stopPhaseCoeff_[4] * pow(t - tHit_, 2);

		x_ = xHit_ + z_ * vecDir2_;
		dx_dt_ = dz_dt_ * vecDir2_;
	} else {
		z_ = stopPhaseCoeff_[1] * (tStop_ - tHit_)
				+ stopPhaseCoeff_[3] * pow(tStop_ - tHit_, 3)
				+ stopPhaseCoeff_[4] * pow(tStop_ - tHit_, 4);
		dz_dt_ = stopPhaseCoeff_[1]
				+ 3 * stopPhaseCoeff_[3] * pow(tStop_ - tHit_, 2)
				+ 4 * stopPhaseCoeff_[4] * pow(tStop_ - tHit_, 3);

		x_ = xHit_ + z_ * vecDir2_;
		dx_dt_ = dz_dt_ * vecDir2_;
	}
	viaPoint_.transforms[0].translation.x = x_[0];
	viaPoint_.transforms[0].translation.y = x_[1];
	viaPoint_.transforms[0].translation.z = height_;
	viaPoint_.velocities[0].linear.x = dx_dt_[0];
	viaPoint_.velocities[0].linear.y = dx_dt_[1];
	viaPoint_.velocities[0].linear.z = 0.0;
}
