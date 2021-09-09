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

#include <kalman/LinearizedMeasurementModel.hpp>

#include "air_hockey_puck_tracker/ObservationModel.hpp"

namespace air_hockey_baseline_agent {

ObservationModel::ObservationModel() {
	this->V.setIdentity();
}

Measurement ObservationModel::h(const State &x) const {
	Measurement measurement;
	measurement.block<2, 1>(0, 0) = x.block<2, 1>(0, 0);
	measurement.theta() = x.theta();
	return measurement;
}

Kalman::Jacobian<Measurement, PuckState>& ObservationModel::getH() {
	return this->H;
}

void ObservationModel::updateJacobians(const State &x) {
	// H = dh/dx (Jacobian of measurement function w.r.t. the state)
	this->H.setZero();

	// partial derivative of meas.x() w.r.t. x.x()
	this->H(Measurement::X, State::X) = 1.0;
	this->H(Measurement::Y, State::Y) = 1.0;
	this->H(Measurement::THETA, State::THETA) = 1.0;
}

void ObservationModel::setMeasurementNoise(Kalman::Jacobian<Measurement, Measurement> &noise){
    this->V = noise;
}

}

