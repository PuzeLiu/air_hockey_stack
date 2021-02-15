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

#include "air_hockey_puck_tracker/SystemModel.hpp"

namespace air_hockey_baseline_agent {
    class EKF_Wrapper;

    class Measurement : public Kalman::Vector<double, 3> {
    public:
        KALMAN_VECTOR(Measurement, double, 3)

        //! X-position
        static constexpr size_t X = 0;
        //! Y-position
        static constexpr size_t Y = 1;
        //! Yaw-angle
        static constexpr size_t THETA = 2;

        inline double x() const { return (*this)[X]; }

        inline double y() const { return (*this)[Y]; }

        inline double theta() const { return (*this)[THETA]; }

        inline double &x() { return (*this)[X]; }

        inline double &y() { return (*this)[Y]; }

        inline double &theta() { return (*this)[THETA]; }
    };

    class ObservationModel : public Kalman::LinearizedMeasurementModel<PuckState, Measurement> {
    public:
        friend class air_hockey_baseline_agent::EKF_Wrapper;
        ObservationModel();


        Measurement h(const PuckState &x) const;
        Kalman::Jacobian<Measurement, PuckState>& getH();
    protected:

        void updateJacobians(const PuckState &x);
    };
}
#endif //PUCK_TRACKER_OBSERVATIONMODEL_H
