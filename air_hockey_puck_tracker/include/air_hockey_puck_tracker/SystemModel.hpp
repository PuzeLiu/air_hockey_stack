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

namespace air_hockey_baseline_agent {
typedef Kalman::Vector<double, 2> Vector2;

/**
 * @brief System state vector-type for the puck
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and (x,y)-Velocity.
 *
 * @param T Numeric scalar type
 */

class PuckState: public Kalman::Vector<double, 6> {
public:KALMAN_VECTOR(PuckState, double, 6)

	//! X-position
	static constexpr size_t X = 0;
	//! Y-position
	static constexpr size_t Y = 1;
	//! X-velocity
	static constexpr size_t DX = 2;
	//! Y-velocity
	static constexpr size_t DY = 3;
	//! Yaw
	static constexpr size_t THETA = 4;
	//! Yaw-angular-velocity
	static constexpr size_t DTHETA = 5;

	inline double x() const {
		return (*this)[X];
	}
	inline double y() const {
		return (*this)[Y];
	}
	inline double dx() const {
		return (*this)[DX];
	}
	inline double dy() const {
		return (*this)[DY];
	}
	inline double theta() const {
		return (*this)[THETA];
	}
	inline double dtheta() const {
		return (*this)[DTHETA];
	}

	inline double& x() {
		return (*this)[X];
	}
	inline double& y() {
		return (*this)[Y];
	}
	inline double& dx() {
		return (*this)[DX];
	}
	inline double& dy() {
		return (*this)[DY];
	}
	inline double& theta() {
		return (*this)[THETA];
	}
	inline double& dtheta() {
		return (*this)[DTHETA];
	}
};

/**
 * @brief System control-input vector-type for the puck
 *
 * This is the system control-input of the puck, here
 * is the time step dt.
 *
 * @param T Numeric scalar type
 */
class Control: public Kalman::Vector<double, 1> {
public:KALMAN_VECTOR(Control, double, 1)

	//! Velocity
	static constexpr size_t DT = 0;

	inline double dt() const {
		return (*this)[DT];
	}

	inline double& dt() {
		return (*this)[DT];
	}
};

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
	//! State type shortcut definition
	typedef air_hockey_baseline_agent::PuckState S;

	//! Control type shortcut definition
	typedef air_hockey_baseline_agent::Control C;

	//! Friction coefficient for air drag
	double damping;
	//! Friction coefficient for sliding movement
	double mu;

	/**
	 * Constructor of the dynamic model
	 * @param c Friction coefficient for air
	 * @param d Friction coefficient for lateral movement
	 */
	SystemModel(double damping, double friction);

    void setDamping(double damping);

    void setMu(double mu);

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

    void setSystemNoise(Kalman::Jacobian<State, State> noise);

protected:
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
	void updateJacobians(const S &x, const C &u);
};

}
#endif //PUCK_TRACKER_SYSTEMMODEL_H
