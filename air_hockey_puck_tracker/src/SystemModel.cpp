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

#include "air_hockey_puck_tracker/SystemModel.hpp"

namespace AirHockey {

SystemModel::SystemModel(double c, double d) {
	m_c = c;
	m_d = d;
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
	S x_;

	x_.x() = x.x() + u.dt() * x.dx();
	x_.y() = x.y() + u.dt() * x.dy();
	x_.theta() = x.theta() + u.dt() * x.dtheta();
	if (x.dx() != 0 && x.dy() != 0) {
		x_.dx() = x.dx() - u.dt() * m_c * x.dx()
				- u.dt() * m_d * x.dx() / abs(x.dx());
		x_.dy() = x.dy() - u.dt() * m_c * x.dy()
				- u.dt() * m_d * x.dy() / abs(x.dy());
	} else {
		x_.dx() = x.dx() - u.dt() * m_c * x.dx();
		x_.dy() = x.dy() - u.dt() * m_c * x.dy();
	}

	x_.theta() = x.theta() + u.dt() * x.dtheta();
	x_.dtheta() = x.dtheta();
	// Return transitioned state vector
	return x_;
}

void SystemModel::updateJacobians(const S &x, const C &u) {
	this->F.setIdentity();
	this->F(S::X, S::DX) = u.dt();
	this->F(S::Y, S::DY) = u.dt();
	this->F(S::DX, S::DX) = 1. - u.dt() * m_c;
	this->F(S::DY, S::DY) = 1. - u.dt() * m_c;
	this->F(S::THETA, S::DTHETA) = u.dt();
}

}
