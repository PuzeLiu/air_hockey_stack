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

namespace air_hockey_baseline_agent {

    static double cross2D(Vector2 v1, Vector2 v2) {
        return v1.x() * v2.y() - v1.y() * v2.x();
    }

    AirHockeyTable::AirHockeyTable(double length, double width, double goalWidth, double puckRadius,
		double restitution, double rimFriction, double dt) {
        m_length = length;
        m_width = width;
        m_puckRadius = puckRadius;
        m_goalWidth = goalWidth;

        m_e = restitution;
		m_rimFriction = rimFriction;
        m_dt = dt;

        Vector2 ref, offsetP1, offsetP2, offsetP3, offsetP4;
        ref << length / 2, 0;
        offsetP1 << -(m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
        offsetP2 << -(m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
        offsetP3 << (m_length / 2 - m_puckRadius), -(m_width / 2 - m_puckRadius);
        offsetP4 << (m_length / 2 - m_puckRadius), (m_width / 2 - m_puckRadius);
        offsetP1 += ref;
        offsetP2 += ref;
        offsetP3 += ref;
        offsetP4 += ref;

        m_boundary.row(0) << offsetP1.x(), offsetP1.y(), offsetP3.x(), offsetP3.y();
        m_boundary.row(1) << offsetP3.x(), offsetP3.y(), offsetP4.x(), offsetP4.y();
        m_boundary.row(2) << offsetP4.x(), offsetP4.y(), offsetP2.x(), offsetP2.y();
        m_boundary.row(3) << offsetP2.x(), offsetP2.y(), offsetP1.x(), offsetP1.y();

		collisionRim = -1;

		m_jacCollision.setIdentity();

		Eigen::Matrix<double,6, 6> T_tmp;
		// First Rim
		T_tmp.setIdentity();
		m_rimGlobalTransforms.push_back(T_tmp);
		m_rimGlobalTransformsInv.push_back(T_tmp.inverse());
		// Second Rim
		T_tmp.setZero();
		T_tmp(0,1) = -1.;
		T_tmp(1, 0) = 1.;
		T_tmp(2,3) = -1.;
		T_tmp(3,2) = 1.;
		T_tmp(4,4) = 1.;
		T_tmp(5,5) = 1.;
		m_rimGlobalTransforms.push_back(T_tmp);
		m_rimGlobalTransformsInv.push_back(T_tmp.inverse());
		// Third Rim
		T_tmp.setZero();
		T_tmp(0,0) = -1.;
		T_tmp(1,1) = -1.;
		T_tmp(2,2) = -1.;
		T_tmp(3,3) = -1.;
		T_tmp(4,4) = 1.;
		T_tmp(5,5) = 1.;
		m_rimGlobalTransforms.push_back(T_tmp);
		m_rimGlobalTransformsInv.push_back(T_tmp.inverse());
		// Fourth Rim
		T_tmp.setZero();
		T_tmp(0,1) = 1.;
		T_tmp(1, 0) = -1.;
		T_tmp(2,3) = 1.;
		T_tmp(3,2) = -1.;
		T_tmp(4,4) = 1.;
		T_tmp(5,5) = 1.;
		m_rimGlobalTransforms.push_back(T_tmp);
		m_rimGlobalTransformsInv.push_back(T_tmp.inverse());
    }

    AirHockeyTable::~AirHockeyTable() {

    }

	void AirHockeyTable::setDynamicsParameter(double restitution, double rimFriction)
	{
		m_e = restitution;
		m_rimFriction = rimFriction;
	}

    bool AirHockeyTable::applyCollision(PuckState &state, jacobianType &jacobian) {
		collisionRim = -1;
        Vector2 p, vel;
        p = state.block<2, 1>(0, 0);
        vel = state.block<2, 1>(2, 0);

        if (abs(p.y()) < m_goalWidth / 2  && p.x() < m_boundary(0, 0) + m_puckRadius) {
            return false;
        } else if (abs(p.y()) < m_goalWidth / 2 && p.x() > m_boundary(0, 2) - m_puckRadius) {
            return false;
        }

        Vector2 u = vel * m_dt;
		Vector2 p1, p2, v, w;

        for (int i = 0; i < m_boundary.rows(); ++i) {
			p1 = m_boundary.block<1, 2>(i, 0);
			p2 = m_boundary.block<1, 2>(i, 2);
			v = p2 - p1;
			w = p1 - p;

            double denominator = cross2D(v, u);
            double s = cross2D(v, w) / denominator;
            double r = cross2D(u, w) / denominator;
            if (abs(denominator) < 1e-6) {
                continue;
            }
            if (cross2D(w, v) < 0
                || (s >= 0 + 1e-4 && s <= 1 - 1e-4 && r >= 0 + 1e-4
                    && r <= 1 - 1e-4)) {

				double theta = state.theta();
				double dtheta = state.dtheta();
				collisionRim = i;

				Eigen::Rotation2D<double> rot(M_PI_2);
				Vector2 vecT = v / v.norm();
				Vector2 vecN = rot * vecT;

				double vtScalar = vel.dot(vecT);
				double vnScalar = vel.dot(vecN);
				double vtNextScalar;
				double vnNextScalar;

				if (abs(vtScalar + m_puckRadius * dtheta) < 3 * m_rimFriction * (1 + m_e) * abs(vnScalar))
				{
					// Velocity on next time step without sliding
					vtNextScalar = (2. / 3.) * vtScalar - (m_puckRadius / 3) * dtheta;
					vnNextScalar = -m_e * vnScalar;
					// Angular Velocity next point
					state.dtheta() = 1. / 3. * dtheta - 2. / (3. * m_puckRadius) * vtScalar;

					// Update jacobian
					m_jacCollision.setIdentity();
					m_jacCollision(0, 2) = m_dt;
					m_jacCollision(1, 3) = m_dt;
					m_jacCollision(2, 2) = 2. / 3.;
					m_jacCollision(2, 5) = - m_puckRadius / 3.;
					m_jacCollision(3, 3) = -m_e;
					m_jacCollision(4, 5) = m_dt;
					m_jacCollision(5, 2) = -2. / (3. * m_puckRadius);
					m_jacCollision(5, 5) = 1. / 3.;
					jacobian = m_rimGlobalTransformsInv[i] * m_jacCollision * m_rimGlobalTransforms[i];
				}
				else
				{
					// Velocity on next time step with sliding
					slideDir = copysign(1, vtScalar + dtheta * m_puckRadius);
					vtNextScalar = vtScalar + m_rimFriction * slideDir * (1 + m_e) * vnScalar;
					vnNextScalar = -m_e * vnScalar;
					state.dtheta() = dtheta + 2. * m_rimFriction * slideDir * (1 + m_e) / (m_puckRadius) * vnScalar;

					// Update jacobian
					m_jacCollision.setIdentity();
					m_jacCollision(0, 2) = m_dt;
					m_jacCollision(1, 3) = m_dt;
					m_jacCollision(2, 3) = m_rimFriction * slideDir * (1 + m_e);
					m_jacCollision(3, 3) = -m_e;
					m_jacCollision(4, 5) = m_dt;
					m_jacCollision(5, 3) = m_jacCollision(2, 3) * 2 / m_puckRadius;
					jacobian = m_rimGlobalTransformsInv[i] * m_jacCollision * m_rimGlobalTransforms[i];
				}

				// Linear Velocity next point
				state.block<2, 1>(2, 0) = vnNextScalar * vecN + vtNextScalar * vecT;

				// Position of next point
				state.block<2, 1>(0, 0) = p + s * u + (1 - s) * state.block<2, 1>(2, 0) * m_dt;
				// Angular Position of next point
				state.theta() = theta + s * dtheta * m_dt + (1 - s) * state.dtheta() * m_dt;

                return true;
            }
        }
        return false;
    }

	Mallet::Mallet(double puckRadius, double malletRadius, double restitution, double dt) {
		m_dt = dt;
		t_prev = 0.;
		m_malletState.setZero();

		m_radiusPuck = puckRadius;
		m_radiusMallet = malletRadius;
		m_e = restitution;
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
		}
	}

	bool Mallet::applyCollision(PuckState &puckState, jacobianType &jacobian) {
		Vector2 pPuck = puckState.block<2, 1>(0, 0);
		Vector2 vPuck = puckState.block<2, 1>(2, 0);

		Vector2 pMallet = m_malletState.block<2, 1>(0, 0);
		Vector2 vMallet = m_malletState.block<2, 1>(2, 0);

		Vector2 vRelative = m_dt * vPuck;
		Vector2 pRelative = pMallet - pPuck;

		double distToGo = pRelative.norm() - m_radiusPuck - m_radiusMallet;
		double t = distToGo / (vRelative.dot(pRelative) / pRelative.norm());

		if (t > 0 && t < 1) {
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

	void Mallet::setDynamicsParameter(double malletRes) {
		m_e = malletRes;
	}

	CollisionModel::CollisionModel(double tableLength, double tableWidth, double goalWidth,	double puckRadius,
		double malletRadius, double &restitutionTable, double &restitutionMallet, double &rimFriction, double dt)
		: m_table(tableLength, tableWidth, goalWidth,
			puckRadius, restitutionTable, rimFriction, dt),
			m_mallet(puckRadius, malletRadius,restitutionMallet, dt) {

	}

	bool CollisionModel::applyCollision(PuckState &puckState, const bool &checkMallet, jacobianType &jacobian) {
		if (checkMallet) {
			return m_mallet.applyCollision(puckState, jacobian);
		}
		return m_table.applyCollision(puckState, jacobian);
	}

	void CollisionModel::setTableDynamicsParam(const double tableRes, const double rimFriction)
	{
		m_table.setDynamicsParameter(tableRes, rimFriction);
	}

	void CollisionModel::setMalletDynamicsParam(const double malletRes)
	{
		m_mallet.setDynamicsParameter(malletRes);
	}
}
