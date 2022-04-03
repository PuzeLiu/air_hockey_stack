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

#include <rosconsole/macros_generated.h>
#include "air_hockey_puck_tracker/SystemModel.hpp"

namespace air_hockey_baseline_agent {

    SystemModel::SystemModel(double tableDamping_, double tableFriction_, double tableLength_, double tableWidth_,
		double goalWidth_, double puckRadius_, double malletRadius_,
		double tableRes_, double malletRes_, double rimFriction_, double dt_) :
		tableDamping(tableDamping_), tableFriction(tableFriction_), tableLength(tableLength_), tableWidth(tableWidth_),
		goalWidth(goalWidth_), puckRadius(puckRadius_), malletRadius(malletRadius_),
		tableRes(tableRes_), malletRes(malletRes_), rimFriction(rimFriction_), dt(dt_){

        collisionModel = new CollisionModel(tableLength, tableWidth, goalWidth, puckRadius, malletRadius,
                                             tableRes, malletRes, rimFriction, dt);

		J_linear.setIdentity();
		J_linear(S::X, S::DX) = dt;
		J_linear(S::Y, S::DY) = dt;
		J_linear(S::DX, S::DX) = 1. - dt * tableDamping;
		J_linear(S::DY, S::DY) = 1. - dt * tableDamping;
		J_linear(S::THETA, S::DTHETA) = dt;
		J_linear(S::DTHETA, S::DTHETA) = 1.;
    }

    SystemModel::~SystemModel(){
        delete collisionModel;
    }

/**
 * @brief Definition of (non-linear) state transition function
 *
 * This function defines how the system state is propagated through time,
 * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
 * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
 * the system control input \f$u\f$.
 *
 * @param [in] x The system state in current time-step
 * @param [in] u The control vector input
 * @returns The (predicted) system state in the next time-step
 */
    SystemModel::S SystemModel::f(const S &x, const C &u) const {
        //! Predicted state vector after transition
        S x_ = x;

		x_.block<2, 1>(0, 0) = x.block<2, 1>(0, 0)
			+ u.dt() * x.block<2, 1>(2, 0);

		if (x.block<2, 1>(2, 0).norm() > 1e-6) {
			x_.block<2, 1>(2, 0) = x.block<2, 1>(2, 0)
				- u.dt() * (tableDamping * x.block<2, 1>(2, 0) +
					tableFriction * x.block<2, 1>(2, 0).cwiseSign());
		} else {
			x_.block<2, 1>(2, 0) = x.block<2, 1>(2, 0) - u.dt() * tableDamping * x.block<2, 1>(2, 0);
		}
		double angle = fmod(x.theta() + u.dt() * x.dtheta(), M_PI * 2);
		if (angle > M_PI) angle -= M_PI * 2;
		else if (angle < -M_PI) angle += M_PI * 2;

		x_.theta() = angle;
		x_.dtheta() = x.dtheta();

        // Return transitioned state vector
        return x_;
    }

    void SystemModel::updateJacobians(S &x, const C &u) {
		this->F = J_linear;
    }

    void SystemModel::setDamping(double damping_) {
        tableDamping = damping_;
    }

    void SystemModel::setTableFriction(double mu_) {
        tableFriction = mu_;
    }

    void SystemModel::setTableDynamicsParam(const double tableRes_, const double rimFriction_) {
		tableRes = tableRes_;
		rimFriction = rimFriction_;
        collisionModel->setTableDynamicsParam(tableRes, rimFriction);
    }

    void SystemModel::setMalletDynamicsParam(const double malletRestitution) {
		malletRes = malletRestitution;
        collisionModel->setMalletDynamicsParam(malletRes);
    }

    bool SystemModel::isOutsideBoundary(const Measurement &measurement) const {
		if (abs(measurement.y()) > tableWidth / 2 - puckRadius + 0.01 ||
			measurement.x() < -0.01 ||
			measurement.x() > tableLength - puckRadius + 0.01) {
			return true;
		}
		return false;
    }

	bool SystemModel::isOutsideBoundary(const PuckState &state) const {
		if (abs(state.y()) > tableWidth / 2 - puckRadius + 0.02 ||
			state.x() < -0.01 ||
			state.x() > tableLength - puckRadius + 0.02) {
			return true;
		}
		return false;
	}

    void SystemModel::updateMalletState(geometry_msgs::TransformStamped stamped) {
        collisionModel->m_mallet.setState(stamped);
    }


}
