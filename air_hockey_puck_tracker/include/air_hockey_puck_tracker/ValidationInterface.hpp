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

#ifndef PUCK_TRACKER_VALIDATION_HPP
#define PUCK_TRACKER_VALIDATION_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>
#include "air_hockey_puck_tracker/InnovationMsg.h"
#include "SystemModel.hpp"


namespace air_hockey_baseline_agent{
    class ValidationInterface{
        ros::NodeHandle m_nh;
        ros::Publisher m_pub_predict;
        ros::Publisher m_pub_true;
        ros::Publisher m_pub_one;
        air_hockey_puck_tracker::InnovationMsg m_msg;

    public:
        ValidationInterface(const ros::NodeHandle &mNh);

        void publishPrediction(const PuckState& prediction, ros::Time stamp, double predictedTime);

        void publishMeasurement(const PuckState& measurement);

        void publishOneStepPrediction(const PuckState& prediction, double predictedTime);

        void record(const PuckState& prediction, const PuckState& measurement);

    };
}

#endif //PUCK_TRACKER_VALIDATION_HPP
