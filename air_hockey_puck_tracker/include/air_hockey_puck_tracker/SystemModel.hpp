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

#ifndef PUCK_TRACKER_SYSTEMMODEL_H
#define PUCK_TRACKER_SYSTEMMODEL_H

#include <kalman/LinearizedSystemModel.hpp>
#include "CollisionModel.hpp"
#include "PuckState.hpp"

namespace air_hockey_baseline_agent {

/**
 * @brief System model for the puck movement
 *
 * This is the system model defining how puck moves from one time-step to the
 * next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
class SystemModel: public Kalman::LinearizedSystemModel<PuckState, Control> {
public:
	friend class air_hockey_baseline_agent::EKF_Wrapper;
	//! State type shortcut definition
	typedef air_hockey_baseline_agent::PuckState S;

	//! Control type shortcut definition
	typedef air_hockey_baseline_agent::Control C;

	//! Dimensions of the table and puck
	double tableLength, tableWidth, puckRadius, malletRadius, goalWidth, dt;

    //! Friction coefficient of the table surface
	double tableDamping, tableFriction;

	double tableRes, malletRes, rimFriction;

	Eigen::Matrix<double,6, 6> J_linear;

    /**
	 * Constructor of the dynamic model
	 * @param c Friction coefficient for air
	 * @param d Friction coefficient for lateral movement
	 */
    SystemModel(double tableDamping, double tableFriction, double tableLength, double tableWidth,
                double goalWidth, double puckRadius, double malletRadius, double tableRestitution,
                double malletRestitution, double rimFriction, double dt);

	~SystemModel();

    void setDamping(double damping);

    void setTableFriction(double mu);

    void setTableDynamicsParam(const double tableRes, const double rimFriction);

    void setMalletDynamicsParam(const double malletRes);

    bool isOutsideBoundary(const Measurement &measurement) const;

	bool isOutsideBoundary(const PuckState &puck_state) const;

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
	S f(const S &x, const C &u) const;

    void updateMalletState(geometry_msgs::TransformStamped stamped);

	/**
	 * @brief Update jacobian matrices for the system state transition function using current state
	 *
	 * This will re-compute the (state-dependent) elements of the jacobian matrices
	 * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
	 * current state \f$x\f$.
	 *
	 * @note This is only needed when implementing a LinearizedSystemModel,
	 *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
	 *       When using a fully non-linear filter such as the UnscentedKalmanFilter
	 *       or its square-root form then this is not needed.
	 *
	 * @param x The current system state around which to linearize
	 * @param u The current system control input
	 */
	void updateJacobians(S &x, const C &u);

private:
    //! Model for collision
    CollisionModel *collisionModel;
};

}
#endif //PUCK_TRACKER_SYSTEMMODEL_H
