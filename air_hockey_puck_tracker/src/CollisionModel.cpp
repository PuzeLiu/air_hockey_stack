/*
 * MIT License
 * Copyright (c) 2020 Davide Tateo, Puze Liu
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

#include "air_hockey_puck_tracker/CollisionModel.hpp"

using namespace std;

namespace AirHockey {

static double cross2D(Vector2 v1, Vector2 v2) {
	return v1.x() * v2.y() - v1.y() * v2.x();
}

AirHockeyTable::AirHockeyTable(double e, double dt) {
	m_center.setZero();
	m_yaw = 0.;
	m_length = 1.96;
	m_width = 1.04;
	m_puckRadius = 0.03165;

	m_e = e;
	m_dt = dt;
}

AirHockeyTable::~AirHockeyTable() {
}

void AirHockeyTable::setTransform(const geometry_msgs::TransformStamped &transform) {
	m_center.x() = transform.transform.translation.x;
	m_center.y() = transform.transform.translation.y;
	m_z = transform.transform.translation.z;
	Eigen::Quaterniond quat;
	quat.x() = transform.transform.rotation.x;
    quat.y() = transform.transform.rotation.y;
    quat.z() = transform.transform.rotation.z;
    quat.w() = transform.transform.rotation.w;
    m_yaw = quat.toRotationMatrix().eulerAngles(0, 1, 2)[2];

    Eigen::Rotation2D<double> rotation(m_yaw);

	Vector2 vecTmp, offsetP1, offsetP2, offsetP3, offsetP4;
	vecTmp << -(m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
	offsetP1 = m_center + rotation * vecTmp;
	vecTmp << -(m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
	offsetP2 = m_center + rotation * vecTmp;
	vecTmp << (m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
	offsetP3 = m_center + rotation * vecTmp;
	vecTmp << (m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
	offsetP4 = m_center + rotation * vecTmp;
	m_boundary.row(0) << offsetP1.x(), offsetP1.y(), offsetP3.x(), offsetP3.y();
	m_boundary.row(1) << offsetP3.x(), offsetP3.y(), offsetP4.x(), offsetP4.y();
	m_boundary.row(2) << offsetP4.x(), offsetP4.y(), offsetP2.x(), offsetP2.y();
	m_boundary.row(3) << offsetP2.x(), offsetP2.y(), offsetP1.x(), offsetP1.y();
}

bool AirHockeyTable::applyCollision(EKF_Wrapper::State &state) {
	Vector2 p, vel;
	p = state.block<2, 1>(0, 0);
	vel = state.block<2, 1>(2, 0);
	double theta = state.theta();
	double dtheta = state.dtheta();

	for (int i = 0; i < m_boundary.rows(); ++i) {
		Vector2 p1 = m_boundary.block<1, 2>(i, 0);
		Vector2 p2 = m_boundary.block<1, 2>(i, 2);

		Vector2 u = vel * m_dt;
		Vector2 v = p2 - p1;
		Vector2 w = p1 - p;

		Eigen::Rotation2D<double> rot(M_PI_2);
		Vector2 vecT = v / v.norm();
		Vector2 vecN = rot * vecT;

		double denominator = cross2D(v, u);
		if (abs(denominator) < 1e-6) {
			// parallel
			continue;
		} else {
			double s = cross2D(v, w) / denominator;
			double r = cross2D(u, w) / denominator;
			if (cross2D(w, v) < 0
					|| s >= 0 + 1e-4 && s <= 1 - 1e-4 && r >= 0 + 1e-4
							&& r <= 1 - 1e-4) {
			    boost::algorithm::clamp(s, 0, 1);
                boost::algorithm::clamp(r, 0, 1);

				double vtScalar = vel.dot(vecT);
				double vnScalar = vel.dot(vecN);

				// Velocity on next time step
				double vtNextScalar = 2. / 3. * vtScalar - m_puckRadius / 5 * dtheta; // - m_puckRadius / 3 * dtheta;
				double vnNextScalar = -m_e * vnScalar;
				double dthetaNext = 1. / 3. * dtheta
						- 2. / (3. * m_puckRadius) * vtScalar;
				Vector2 vNext = vnNextScalar * vecN + vtNextScalar * vecT;

				// Position of intersection point
				Vector2 pInter = p + s * u;
				// Angular Position of intersection point
				double thetaInter = theta + s * dtheta * m_dt;
				// Position of next point
				Vector2 pNext = pInter + (1 - s) * vNext * m_dt;
				// Angular Position of next point
				double thetaNext = thetaInter + (1 - s) * dthetaNext * m_dt;

				state.block<2, 1>(0, 0) = pNext;
				state.block<2, 1>(2, 0) = vNext;
				state.theta() = thetaNext;
				state.dtheta() = dthetaNext;
				return true;
			}
		}
	}
	return false;
}

bool AirHockeyTable::isOutsideBoundary(Measurement &measurement) {
    for (int i = 0; i < m_boundary.rows(); ++i) {
        Vector2 p1 = m_boundary.block<1, 2>(i, 0);
        Vector2 p2 = m_boundary.block<1, 2>(i, 2);

        Vector2 pPuck = measurement.block<2, 1>(0, 0);
        if (cross2D(p2 - p1, pPuck - p1) < -1e-6){
            return true;
        }
    }
    return false;
}

    Mallet::Mallet(double e, double dt) {
	m_dt = dt;
	t_prev = 0.;
	m_malletState.setZero();
	m_malletStatePredict.setZero();

	m_radiusPuck = 0.03165;
	m_radiusMallet = 0.04815;
	m_e = 0.5;
}

void Mallet::setState(const geometry_msgs::TransformStamped &tfMallet) {
	double deltaT = tfMallet.header.stamp.toSec() - t_prev;
    if (deltaT > 0) {
        m_malletState.dx() = (tfMallet.transform.translation.x - m_malletState.x())
                             / deltaT;
        m_malletState.dy() = (tfMallet.transform.translation.y - m_malletState.y())
                             / deltaT;
        tf2::Quaternion quat;
        quat.setX(tfMallet.transform.rotation.x);
        quat.setY(tfMallet.transform.rotation.y);
        quat.setZ(tfMallet.transform.rotation.z);
        quat.setW(tfMallet.transform.rotation.w);
        tf2::Matrix3x3 rotMat(quat);
        double roll, pitch, yaw;
        rotMat.getEulerYPR(yaw, pitch, roll);

        m_malletState.dtheta() = (yaw - m_malletState.theta()) / deltaT;

        m_malletState.x() = tfMallet.transform.translation.x;
        m_malletState.y() = tfMallet.transform.translation.y;
        m_malletState.theta() = yaw;
        t_prev = tfMallet.header.stamp.toSec();
        m_malletStatePredict = m_malletState;
    }
}

void Mallet::predict() {
	m_malletStatePredict.x() = m_malletStatePredict.x()
			+ m_malletStatePredict.dx() * m_dt;
	m_malletStatePredict.y() = m_malletStatePredict.y()
			+ m_malletStatePredict.dy() * m_dt;
	m_malletStatePredict.theta() = m_malletStatePredict.theta()
			+ m_malletStatePredict.dtheta() * m_dt;
}

bool Mallet::applyCollision(State &puckState) {
	Vector2 pPuck = puckState.block<2, 1>(0, 0);
	Vector2 vPuck = puckState.block<2, 1>(2, 0);

	Vector2 pMallet = m_malletStatePredict.block<2, 1>(0, 0);
    Vector2 vMallet = m_malletStatePredict.block<2, 1>(2, 0);

	Vector2 vRelative = m_dt * vPuck;
	Vector2 pRelative = pMallet - pPuck;

	double distToGo = pRelative.norm() - m_radiusPuck - m_radiusMallet;
	double t = distToGo / (vRelative.dot(pRelative) / pRelative.norm());

	if (t > 0 && t < 1) {
	    ROS_INFO_STREAM("t: " << t);
		Vector2 pPuckCollide = pPuck + t * vRelative;

		Vector2 vecN = pPuckCollide - pMallet;

		Eigen::Rotation2D<double> rot(-M_PI_2);
		vecN.normalize();
		Vector2 vecT = rot * vecN;
		double vtScalarPuck = vPuck.dot(vecT);
		double vnScalarPuck = vPuck.dot(vecN);
		double vtScalarMallet = vMallet.dot(vecT);
		double vnScalarMallet = vMallet.dot(vecN);

		double vtScalarPuckNext = 2. / 3. * vtScalarPuck + 1. / 3. * vtScalarMallet;
		double vnScalarPuckNext = -m_e * vnScalarPuck + (1 + m_e) * vnScalarMallet;

		Vector2 vPuckNext = vtScalarPuckNext * vecT + vnScalarPuckNext * vecN;
		//! Update the state
		puckState.block<2, 1>(0, 0) = pPuckCollide + (1 - t) * vPuckNext * m_dt;
		puckState.block<2, 1>(2, 0) = vPuckNext;
		return true;
	}
	return false;
}

CollisionModel::CollisionModel(geometry_msgs::TransformStamped &tfTable,
		double &restitutionTable, double &restitutionMallet, double dt) :
		m_table(restitutionTable, dt), m_mallet(restitutionMallet, dt) {
	m_table.setTransform(tfTable);
}

bool CollisionModel::applyCollision(State &puckState, const bool& checkMallet) {
    if (checkMallet){
        m_mallet.applyCollision(puckState);
    }
	return m_table.applyCollision(puckState);
}

}
