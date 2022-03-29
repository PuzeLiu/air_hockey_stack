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

    SystemModel::SystemModel(double tableDamping_, double tableFriction_, double tableLength, double tableWidth,
		double goalWidth, double puckRadius, double malletRadius,
		double restitutionTable, double restitutionMallet, double dt) {

        collisionModel = new CollisionModel(tableLength, tableWidth, goalWidth, puckRadius, malletRadius,
                                             restitutionTable, restitutionMallet, dt);
		tableDamping = tableDamping_;
	    tableFriction = tableFriction_;
		tableRes = restitutionTable;
	    pRadius = puckRadius;

		J_collision.setIdentity();
		J_collision(S::X,S::DX) = dt;
		J_collision(S::Y,S::DY) = dt;
		J_collision(S::DX,S::DX) = 2. / 3.;
		J_collision(S::DX, S::DTHETA) = -pRadius / 3;
		J_collision(S::DY,S::DY) = -tableRes;
		J_collision(S::THETA,S::DTHETA) = dt;
		J_collision(S::DTHETA,S::DX) = -2. / (3. * pRadius);
		J_collision(S::DTHETA, S::DTHETA) = 1. / 3.;

		J_linear.setIdentity();
		J_linear(S::X, S::DX) = dt;
		J_linear(S::Y, S::DY) = dt;
		J_linear(S::DX, S::DX) = 1. - dt * tableDamping;
		J_linear(S::DY, S::DY) = 1. - dt * tableDamping;
		J_linear(S::THETA, S::DTHETA) = dt;
		J_linear(S::DTHETA, S::DTHETA) = 1.;

		Eigen::Matrix<double,6, 6> T_tmp;
		// First Rim
		T_tmp.setIdentity();
		J_collision_vec.push_back(T_tmp.inverse() * J_collision * T_tmp);
		// Second Rim
		T_tmp.setZero();
		T_tmp(0,1) = -1.;
		T_tmp(1, 0) = 1.;
		T_tmp(2,3) = -1.;
		T_tmp(3,2) = 1.;
		T_tmp(4,4) = 1.;
		T_tmp(5,5) = 1.;
		J_collision_vec.push_back(T_tmp.inverse() * J_collision * T_tmp);
		// Third Rim
		T_tmp.setZero();
		T_tmp(0,0) = -1.;
		T_tmp(1,1) = -1.;
		T_tmp(2,2) = -1.;
		T_tmp(3,3) = -1.;
		T_tmp(4,4) = 1.;
		T_tmp(5,5) = 1.;
		J_collision_vec.push_back(T_tmp.inverse() * J_collision * T_tmp);
		// Fourth Rim
		T_tmp.setZero();
		T_tmp(0,1) = 1.;
		T_tmp(1, 0) = -1.;
		T_tmp(2,3) = 1.;
		T_tmp(3,2) = -1.;
		T_tmp(4,4) = 1.;
		T_tmp(5,5) = 1.;
		J_collision_vec.push_back(T_tmp.inverse() * J_collision * T_tmp);
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
		if (hasCollision){
			//apply collision if there is one
			collisionModel->applyCollision(x_, false);
		}
		else {
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
		}
        // Return transitioned state vector
        return x_;
    }

    void SystemModel::updateJacobians(const S &x, const C &u) {
		hasCollision = collisionModel->hasCollision(x);
        if (hasCollision){
            this->F = J_collision_vec[collisionModel->m_table.collisionRim];
        }
		else {
            this->F = J_linear;
        }
    }

    void SystemModel::setDamping(double damping_) {
        tableDamping = damping_;
    }

    void SystemModel::setTableFriction(double mu_) {
        tableFriction = mu_;
    }

    void SystemModel::setTableDynamicsParam(const double tableRestitution) {
		tableRes = tableRestitution;
        collisionModel->setTableDynamicsParam(tableRes);
    }

    void SystemModel::setMalletDynamicsParam(const double malletRestitution) {
		malletRes = malletRestitution;
        collisionModel->setMalletDynamicsParam(malletRes);
    }

    bool SystemModel::isOutsideBoundary(Measurement &measurement) {
        return collisionModel->m_table.isOutsideBoundary(measurement);
    }

    void SystemModel::updateMalletState(geometry_msgs::TransformStamped stamped) {
        collisionModel->m_mallet.setState(stamped);
    }


}
