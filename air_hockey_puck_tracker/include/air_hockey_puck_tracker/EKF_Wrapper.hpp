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

#ifndef PUCK_TRACKER_EKF_WRAPPER_HPP
#define PUCK_TRACKER_EKF_WRAPPER_HPP

#include <ros/ros.h>

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>

#include "air_hockey_puck_tracker/ObservationModel.hpp"


namespace air_hockey_baseline_agent {
    class EKF_Wrapper : public Kalman::ExtendedKalmanFilter<PuckState> {
    public:
        typedef Measurement InnovationType;
        typedef Kalman::Covariance<Measurement> InnovationCovariance;

        const PuckState &update(ObservationModel &m, const Measurement &z);

        inline PuckState &getState() {
            return x;
        }

        inline const InnovationCovariance &getInnovationCovariance() {
            return S;
        }

        const InnovationCovariance &updateInnovationCovariance(ObservationModel &m);

        inline const InnovationType &getInnovation() {
            return mu;
        }

    protected:
        using ExtendedKalmanFilter::P;
        using ExtendedKalmanFilter::x;

        //! Innovation
        InnovationType mu;
        //! Innovation Covariance
        InnovationCovariance S;

        void calculateInnovation(ObservationModel &m, const Measurement &z);

    };
}

#endif //PUCK_TRACKER_EKF_WRAPPER_HPP
