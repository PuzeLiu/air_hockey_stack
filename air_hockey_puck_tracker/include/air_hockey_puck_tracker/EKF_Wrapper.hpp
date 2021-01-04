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
#include <tf/transform_listener.h>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>

#include "air_hockey_puck_tracker/ObservationModel.hpp"


namespace AirHockey {
class EKF_Wrapper : public Kalman::ExtendedKalmanFilter<State>{
    public:
        typedef Measurement InnovationType;
        typedef Kalman::Covariance<Measurement> InnovationCovariance;

        inline const State& update(ObservationModel& m, const Measurement& z){
            m.updateJacobians( x );

            // COMPUTE KALMAN GAIN
            // compute innovation covariance
            S = ( m.H * P * m.H.transpose() ) + ( m.V * m.getCovariance() * m.V.transpose() );

            // compute kalman gain
            KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();

            // UPDATE STATE ESTIMATE AND COVARIANCE
            // Update state using computed kalman gain and innovation
            calculateInnovation(m, z);
            x += K * ( mu );

            // Update covariance
            P -= K * m.H * P;

            // return updated state estimate
            return this->getState();
        }

        inline State& getState(){
            return x;
        }

    protected:
        using ExtendedKalmanFilter::P;
        using ExtendedKalmanFilter::x;

        //! Innovation
        InnovationType mu;
        //! Innovation Covariance
        InnovationCovariance S;

        inline void calculateInnovation(ObservationModel& m, const Measurement& z){
            mu = z - m.h(x);
            mu.theta() = atan2(sin(mu.theta()), cos(mu.theta()));
        }

    public:
        inline const InnovationCovariance& getInnovationCovariance() {
            return S;
        }

        inline const InnovationCovariance& updateInnovationCovariance(ObservationModel &m){
            S = ( m.H * P * m.H.transpose() ) + ( m.V * m.getCovariance() * m.V.transpose() );
            return S;
        }

        inline const InnovationType& getInnovation(){
            return mu;
        }
    };
}

#endif //PUCK_TRACKER_EKF_WRAPPER_HPP
