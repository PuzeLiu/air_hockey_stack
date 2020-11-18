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

#ifndef PUCK_TRACKER_OBSERVATIONMODEL_H
#define PUCK_TRACKER_OBSERVATIONMODEL_H

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace AirHockey {
    class EKF;

/**
 * @brief Measurement vector measuring the puck's position and velocity
 *
 * @param T Numeric scalar type
 */
    class Measurement : public Kalman::Vector<T, 3> {
    public:
        KALMAN_VECTOR(Measurement, T, 3)

        //! X-position
        static constexpr size_t X = 0;
        //! Y-position
        static constexpr size_t Y = 1;
        //! Yaw-angle
        static constexpr size_t THETA = 2;

        T x() const { return (*this)[X]; }

        T y() const { return (*this)[Y]; }

        T theta() const { return (*this)[THETA]; }

        T &x() { return (*this)[X]; }

        T &y() { return (*this)[Y]; }

        T &theta() { return (*this)[THETA]; }
    };


    /**
    * @brief Measurement model for measuring the puck's position and velocity
    *
    *
    * @param T Numeric scalar type
    * @param CovarianceBase Class template to determine the covariance representation
    *                       (as covariance matrix (StandardBase) or as lower-triangular
    *                       coveriace square root (SquareRootBase))
    */
    class ObservationModel : public Kalman::LinearizedMeasurementModel<State, Measurement> {
    public:
        friend class AirHockey::EKF;
        /**
         * @brief Constructor
         */
        ObservationModel() {
            this->V.setIdentity();
        }

        /**
         * @brief Definition of (possibly non-linear) measurement function
         *
         * This function maps the system state to the measurement that is expected
         * to be received from the sensor assuming the system is currently in the
         * estimated state.
         *
         * @param [in] x The system state in current time-step
         * @returns The (predicted) sensor measurement for the system state
         */
        Measurement h(const State &x) const {
            Measurement measurement;
            measurement.x() = x.x();
            measurement.y() = x.y();
            measurement.theta() = x.theta();
            return measurement;
        }

        Kalman::Jacobian<Measurement, State>& getH(){
            return this->H;
        }

    protected:

        /**
         * @brief Update jacobian matrices for the system state transition function using current state
         *
         * This will re-compute the (state-dependent) elements of the jacobian matrices
         * to linearize the non-linear measurement function \f$h(x)\f$ around the
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
        void updateJacobians(const State &x) {
            // H = dh/dx (Jacobian of measurement function w.r.t. the state)
            this->H.setZero();

            // partial derivative of meas.x() w.r.t. x.x()
            this->H(Measurement::X, State::X) = 1.0;
            this->H(Measurement::Y, State::Y) = 1.0;
            this->H(Measurement::THETA, State::THETA) = 1.0;
        }
    };
}
#endif //PUCK_TRACKER_OBSERVATIONMODEL_H
