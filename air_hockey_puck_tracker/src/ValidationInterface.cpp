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


#include <ros/package.h>
#include "air_hockey_puck_tracker/ValidationInterface.hpp"


using namespace air_hockey_baseline_agent;

ValidationInterface::ValidationInterface(const ros::NodeHandle &mNh): m_nh(mNh) {
    m_pub_predict = m_nh.advertise<air_hockey_puck_tracker::InnovationMsg>("prediction", 10);
    m_pub_true = m_nh.advertise<air_hockey_puck_tracker::InnovationMsg>("measured", 10);
    m_pub_one = m_nh.advertise<air_hockey_puck_tracker::InnovationMsg>("onestep", 10);
}


void ValidationInterface::publishMeasurement(const PuckState &measurement) {
    m_msg.header.stamp = ros::Time::now();
    m_msg.x = measurement.x();
    m_msg.y = measurement.y();
    m_msg.theta = measurement.theta();
    m_pub_true.publish(m_msg);
}

void ValidationInterface::publishOneStepPrediction(const PuckState &prediction, const double predictionTime) {
    m_msg.header.stamp = ros::Time::now();
    m_msg.x = prediction.x();
    m_msg.y = prediction.y();
    m_msg.theta = prediction.theta();
    m_msg.predictionTime = predictionTime;
    m_pub_one.publish(m_msg);
}

void ValidationInterface::record(const PuckState &prediction, const PuckState &measurement) {
    m_msg.header.stamp = ros::Time::now();
    m_msg.x = prediction.x();
    m_msg.y = prediction.y();
    m_msg.theta = prediction.theta();
    m_pub_predict.publish(m_msg);

    m_msg.x = measurement.x();
    m_msg.y = measurement.y();
    m_msg.theta = measurement.theta();
    m_pub_true.publish(m_msg);
}

void ValidationInterface::publishPrediction(const PuckState &prediction, ros::Time stamp, const double predictedTime) {
    m_msg.header.stamp = ros::Time::now();
    m_msg.currentTime = stamp;
    m_msg.x = prediction.x();
    m_msg.y = prediction.y();
    m_msg.theta = prediction.theta();
    m_msg.predictionTime = predictedTime;
    m_pub_predict.publish(m_msg);
}

