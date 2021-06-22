//
// Created by default on 27.05.21.
//

#ifndef SRC_PARTICLEVISUALIZATIONINTERFACE_HPP
#define SRC_PARTICLEVISUALIZATIONINTERFACE_HPP


#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include "SystemModel.hpp"
#include "EKF_Wrapper.hpp"
#include "ParticleFilter.hpp"

namespace air_hockey_baseline_agent {
    class ParticleVisualizationInterface {
        ros::NodeHandle m_nh;

        ros::Publisher m_particlePub;

        visualization_msgs::MarkerArray m_particleMarkerMsg;
    public:
        void visualize(ParticleFilter& pf, const std::string& refName);
        explicit ParticleVisualizationInterface(const ros::NodeHandle &nh);

    };
}

#endif //SRC_PARTICLEVISUALIZATIONINTERFACE_HPP
