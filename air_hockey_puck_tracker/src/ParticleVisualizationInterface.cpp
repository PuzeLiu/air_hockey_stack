//
// Created by default on 27.05.21.
//

#include <tf2/LinearMath/Quaternion.h>
#include "air_hockey_puck_tracker/ParticleVisualizationInterface.hpp"
#include "air_hockey_puck_tracker/ParticleFilter.hpp"
using namespace air_hockey_baseline_agent;

ParticleVisualizationInterface::ParticleVisualizationInterface(const ros::NodeHandle &nh): m_nh(nh) {
    m_particlePub = m_nh.advertise<visualization_msgs::MarkerArray> ("visualization_marker_array", 10);
}

void ParticleVisualizationInterface::visualize(ParticleFilter& particleFilter, const std::string& tableRefName) {
    ROS_INFO_STREAM("Visualize Particles of Particle Filter");
    for (int i= 0; i < particleFilter.getParticles().size(); i++){
        PuckState state = particleFilter.getParticles().at(i);
        visualization_msgs::Marker marker;
        marker.header.frame_id = tableRefName;
        marker.header.stamp = ros::Time();
        marker.ns = "Particle";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.position.x = state.x();
        marker.pose.position.y = state.y();
        tf2::Quaternion quaternion;
        quaternion.setRPY(0., 0., state.theta());
        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();
        // lifetime duration in sec
        //marker.lifetime = ros::Duration(5);

        m_particleMarkerMsg.markers.push_back(marker);

    }
    m_particlePub.publish(m_particleMarkerMsg);
}