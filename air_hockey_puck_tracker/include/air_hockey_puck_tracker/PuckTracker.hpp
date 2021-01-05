/*
 * MIT License
 * Copyright (c) 2021 Puze Liu, Davide Tateo
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

#ifndef SRC_PUCK_TRACKER_HPP
#define SRC_PUCK_TRACKER_HPP

#include <boost/thread.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "SystemModel.hpp"
#include "ObservationModel.hpp"
#include "CollisionModel.hpp"
#include "EKF_Wrapper.hpp"

namespace AirHockey {
    struct PuckPredictedState{
        State state;
        double time;
    };

    class PuckTracker {
    public:
        PuckTracker(ros::NodeHandle nh, ros::Rate rate);

        ~PuckTracker();

        void start();

        const PuckPredictedState& getPuckState();

    private:
        void init();

        void setCovariance();

        void startTracking();

    private:
        ros::NodeHandle nh_;
        ros::Rate rate_;

        tf2_ros::TransformListener tfListener_;
        tf2_ros::Buffer tfBuffer_;
        std::string robotBaseName_;
        geometry_msgs::TransformStamped tfTableStatic_, tfPuck_, tfPuckPrev_;

        EKF_Wrapper *kalmanFilter_;
        EKF_Wrapper *puckPredictor_;
        SystemModel *systemModel_;
        ObservationModel *observationModel_;
        CollisionModel *collisionModel_;

        Control u_;
        int maxPredictionSteps_;
        Measurement measurement_;

        PuckPredictedState predictedState_;
        double predictedTime_;

        boost::thread thread_;
        boost::mutex predictionMutex_;
    };
}

#endif //SRC_PUCK_TRACKER_HPP
