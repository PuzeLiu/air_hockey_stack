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

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "air_hockey_puck_tracker/VisualizationInterface.hpp"

namespace air_hockey_baseline_agent {

VisualizationInterface::VisualizationInterface(const ros::NodeHandle &nh, const std::string& tableRefName) :
		m_nh(nh) {
	m_markerPub = m_nh.advertise < visualization_msgs::Marker > ("marker", 1);

	m_predictionMarker.header.frame_id = tableRefName;
	m_predictionMarker.ns = "Prediction";
	m_predictionMarker.type = visualization_msgs::Marker::CYLINDER;
	m_predictionMarker.action = visualization_msgs::Marker::MODIFY;
	m_predictionMarker.scale.x = 0.1;
	m_predictionMarker.scale.y = 0.1;
	m_predictionMarker.scale.z = 0.02;
    if (nh.getNamespace() == "/iiwa_front"){
        m_predictionMarker.color.r = 0.0;
        m_predictionMarker.color.g = 0.0;
        m_predictionMarker.color.b = 1.0;
        m_predictionMarker.color.a = 0.2;
    } else {
        m_predictionMarker.color.r = 0.0;
        m_predictionMarker.color.g = 1.0;
        m_predictionMarker.color.b = 0.0;
        m_predictionMarker.color.a = 0.2;
    }

	m_predictionMarker.pose.position.x = 0.0;
	m_predictionMarker.pose.position.y = 0.0;
	m_predictionMarker.pose.position.z = 0.0;
	m_predictionMarker.pose.orientation.w = 1.;
	m_predictionMarker.pose.orientation.x = 0.;
	m_predictionMarker.pose.orientation.y = 0.;
	m_predictionMarker.pose.orientation.z = 0.;
}

void VisualizationInterface::visualize() {
	m_markerPub.publish(m_predictionMarker);
}

void VisualizationInterface::setPredictionMarker(const PuckState &state,
		const EKF_Wrapper::InnovationCovariance &cov, bool usePuckSize) {
	m_predictionMarker.pose.position.x = state.x();
	m_predictionMarker.pose.position.y = state.y();
	m_predictionMarker.pose.position.z = 0.0;
	tf2::Quaternion quaternion;
	quaternion.setRPY(0., 0., state.theta());
	m_predictionMarker.pose.orientation.x = quaternion.x();
	m_predictionMarker.pose.orientation.y = quaternion.y();
	m_predictionMarker.pose.orientation.z = quaternion.z();
	m_predictionMarker.pose.orientation.w = quaternion.w();

	if (usePuckSize){
		m_predictionMarker.scale.x = 0.0633;
		m_predictionMarker.scale.y = 0.0633;
	} else {
		m_predictionMarker.scale.x = std::sqrt(cov(0, 0)) * 1.96;
		m_predictionMarker.scale.y = std::sqrt(cov(1, 1)) * 1.96;
	}
}

void VisualizationInterface::update(const PuckState& puckState,
									const EKF_Wrapper::InnovationCovariance& innovationCov,
									bool usePuckSize) {
	setPredictionMarker(puckState, innovationCov, usePuckSize);
    visualize();
}

}
