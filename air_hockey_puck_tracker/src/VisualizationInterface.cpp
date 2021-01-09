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

namespace AirHockey {

VisualizationInterface::VisualizationInterface(const ros::NodeHandle &nh,
		double tableHeight, std::string robotBaseName) :
		m_nh(nh), robotBaseName_(robotBaseName) {
	m_markerPub = m_nh.advertise < visualization_msgs::Marker > ("marker", 10);

	m_tableMarker.header.frame_id = "Table";
	m_tableMarker.ns = "Table";
	m_tableMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
	m_tableMarker.action = visualization_msgs::Marker::ADD;
	m_tableMarker.scale.x = 1.0;
	m_tableMarker.scale.y = 1.0;
	m_tableMarker.scale.z = 1.0;
	m_tableMarker.mesh_resource =
			"package://air_hockey_description/meshes/air_hockey_table/mesh/table.dae";
	m_tableMarker.mesh_use_embedded_materials = true;
	m_tableMarker.color.r = 0.9;
	m_tableMarker.color.g = 0.9;
	m_tableMarker.color.b = 0.9;
	m_tableMarker.color.a = 1.0;
	m_tableMarker.pose.position.x = 0.;
	m_tableMarker.pose.position.y = 0.;
	m_tableMarker.pose.position.z = 0.;
	m_tableMarker.pose.orientation.w = 1.;
	m_tableMarker.pose.orientation.x = 0.;
	m_tableMarker.pose.orientation.y = 0.;
	m_tableMarker.pose.orientation.z = 0.;

	m_puckMarker.header.frame_id = "Puck";
	m_puckMarker.ns = "Puck";
	m_puckMarker.type = visualization_msgs::Marker::CYLINDER;
	m_puckMarker.action = visualization_msgs::Marker::ADD;
	m_puckMarker.scale.x = 0.0633;
	m_puckMarker.scale.y = 0.0633;
	m_puckMarker.scale.z = 0.02;
	m_puckMarker.color.r = 1.0;
	m_puckMarker.color.g = 0.0;
	m_puckMarker.color.b = 0.0;
	m_puckMarker.color.a = 1.0;
	m_puckMarker.pose.position.x = 0.;
	m_puckMarker.pose.position.y = 0.;
	m_puckMarker.pose.position.z = 0.;
	m_puckMarker.pose.orientation.w = 1.;
	m_puckMarker.pose.orientation.x = 0.;
	m_puckMarker.pose.orientation.y = 0.;
	m_puckMarker.pose.orientation.z = 0.;

	m_puckMarkerIndicator.header.frame_id = "Puck";
	m_puckMarkerIndicator.ns = "Puck_Indicator";
	m_puckMarkerIndicator.type = visualization_msgs::Marker::CYLINDER;
	m_puckMarkerIndicator.action = visualization_msgs::Marker::ADD;
	m_puckMarkerIndicator.scale.x = 0.01;
	m_puckMarkerIndicator.scale.y = 0.01;
	m_puckMarkerIndicator.scale.z = 0.021;
	m_puckMarkerIndicator.color.r = 0.0;
	m_puckMarkerIndicator.color.g = 1.0;
	m_puckMarkerIndicator.color.b = 0.0;
	m_puckMarkerIndicator.color.a = 1.0;
	m_puckMarkerIndicator.pose.position.x = 0.02;
	m_puckMarkerIndicator.pose.position.y = 0.;
	m_puckMarkerIndicator.pose.position.z = 0.;
	m_puckMarkerIndicator.pose.orientation.w = 1.;
	m_puckMarkerIndicator.pose.orientation.x = 0.;
	m_puckMarkerIndicator.pose.orientation.y = 0.;
	m_puckMarkerIndicator.pose.orientation.z = 0.;

	m_malletMarker.header.frame_id = "Mallet";
	m_malletMarker.ns = "Mallet";
	m_malletMarker.type = visualization_msgs::Marker::CYLINDER;
	m_malletMarker.action = visualization_msgs::Marker::MODIFY;
	m_malletMarker.scale.x = 0.0963;
	m_malletMarker.scale.y = 0.0963;
	m_malletMarker.scale.z = 0.065;
	m_malletMarker.color.r = 1.0;
	m_malletMarker.color.g = 0.0;
	m_malletMarker.color.b = 0.0;
	m_malletMarker.color.a = 1.0;
	m_malletMarker.pose.position.x = 0.;
	m_malletMarker.pose.position.y = 0.;
	m_malletMarker.pose.position.z = 0.;
	m_malletMarker.pose.orientation.w = 1.;
	m_malletMarker.pose.orientation.x = 0.;
	m_malletMarker.pose.orientation.y = 0.;
	m_malletMarker.pose.orientation.z = 0.;

	m_predictionMarker.header.frame_id = "world";
	m_predictionMarker.ns = "Prediction";
	m_predictionMarker.type = visualization_msgs::Marker::CYLINDER;
	m_predictionMarker.action = visualization_msgs::Marker::MODIFY;
	m_predictionMarker.scale.x = 0.1;
	m_predictionMarker.scale.y = 0.1;
	m_predictionMarker.scale.z = 0.02;
    if (robotBaseName_ == "F_link_0"){
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
	m_predictionMarker.pose.position.z = m_tableHeight;
	m_predictionMarker.pose.orientation.w = 1.;
	m_predictionMarker.pose.orientation.x = 0.;
	m_predictionMarker.pose.orientation.y = 0.;
	m_predictionMarker.pose.orientation.z = 0.;

	m_tableHeight = tableHeight;

}

void VisualizationInterface::visualize() {
	m_markerPub.publish(m_tableMarker);
	m_markerPub.publish(m_puckMarker);
//	m_markerPub.publish(m_malletMarker);
	m_markerPub.publish(m_predictionMarker);
	m_markerPub.publish(m_puckMarkerIndicator);
}

void VisualizationInterface::setPredictionMarker(const State &state,
		const EKF_Wrapper::InnovationCovariance &cov) {
	m_predictionMarker.pose.position.x = state.x();
	m_predictionMarker.pose.position.y = state.y();
	m_predictionMarker.pose.position.z = m_tableHeight;
	tf2::Quaternion quaternion;
	quaternion.setRPY(0., 0., state.theta());
	m_predictionMarker.pose.orientation.x = quaternion.x();
	m_predictionMarker.pose.orientation.y = quaternion.y();
	m_predictionMarker.pose.orientation.z = quaternion.z();
	m_predictionMarker.pose.orientation.w = quaternion.w();

	m_predictionMarker.scale.x = std::sqrt(cov(0, 0)) * 1.96;
	m_predictionMarker.scale.y = std::sqrt(cov(1, 1)) * 1.96;
}

    void VisualizationInterface::update(EKF_Wrapper& puckPredictor, ObservationModel& observationModel) {
    m_predictionMarker.header.frame_id = robotBaseName_;
    puckPredictor.updateInnovationCovariance(observationModel);
    setPredictionMarker(puckPredictor.getState(), puckPredictor.getInnovationCovariance());
    visualize();
}

}
