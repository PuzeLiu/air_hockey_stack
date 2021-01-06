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


#ifndef PUCK_TRACKER_VISUALIZATION_INTERFACE_H
#define PUCK_TRACKER_VISUALIZATION_INTERFACE_H

#include <visualization_msgs/Marker.h>
#include "air_hockey_puck_tracker/EKF_Wrapper.hpp"
#include "air_hockey_puck_tracker/PuckTracker.hpp"


namespace AirHockey{
class PuckTracker;
class VisualizationInterface{
public:
    VisualizationInterface(const ros::NodeHandle& nh, double tableHeight);

    void update(PuckTracker& predictor);

private:
    void visualize();
    void setPredictionMarker(const State& state,
                             const EKF_Wrapper::InnovationCovariance& cov);

private:
    ros::NodeHandle m_nh;

    ros::Publisher m_markerPub;

    visualization_msgs::Marker m_tableMarker, m_puckMarker, m_puckMarkerIndicator, m_malletMarker, m_predictionMarker;

    double m_tableHeight;
};

}

#endif //PUCK_TRACKER_VISUALIZATION_INTERFACE_H
