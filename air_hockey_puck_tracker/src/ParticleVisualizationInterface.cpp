//
// Created by default on 27.05.21.
//

#include <tf2/LinearMath/Quaternion.h>
#include "air_hockey_puck_tracker/ParticleVisualizationInterface.hpp"
#include "air_hockey_puck_tracker/ParticleFilter.hpp"
using namespace air_hockey_baseline_agent;

ParticleVisualizationInterface::ParticleVisualizationInterface(const ros::NodeHandle &nh): m_nh(nh) {
    m_particlePub = m_nh.advertise<visualization_msgs::Marker> ("visualization_particle", 1);
}

void ParticleVisualizationInterface::visualize(ParticleFilter& particleFilter, const std::string& tableRefName) {
    m_marker.header.frame_id = tableRefName;
    m_marker.header.stamp = ros::Time();
    m_marker.ns = "Particles";
    m_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    m_marker.action = visualization_msgs::Marker::MODIFY;
    m_marker.scale.x = 0.02;
    m_marker.scale.y = 0.02;
    m_marker.scale.z = 0.02;
    m_marker.color.r = 1.0;
    m_marker.color.g = 1.0;
    m_marker.color.b = 0.0;
    m_marker.color.a = 1.0;

    for (PuckState state: particleFilter.getParticles()){
        geometry_msgs::Point point;
        point.x = state.x();
        point.y = state.y();
        m_marker.points.push_back(point);

    }
    m_particlePub.publish(m_marker);
    m_marker.points.clear();
}