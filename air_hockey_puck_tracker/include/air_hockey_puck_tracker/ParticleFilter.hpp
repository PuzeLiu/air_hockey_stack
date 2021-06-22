//
// Created by default on 26.05.21.
//

#ifndef SRC_PARTICLEFILTER_HPP
#define SRC_PARTICLEFILTER_HPP

#include <vector>
#include "SystemModel.hpp"
#include "ObservationModel.hpp"


namespace air_hockey_baseline_agent {
    class ParticleFilter {
        std::vector<PuckState> particles;
        SystemModel systemModel_;
    public:
        const int sampleSize = 10;
        void sampleParticles(PuckState& state, Eigen::Matrix<double, 6,6>& covariance);
        std::vector<PuckState> getParticles();
        ParticleFilter(SystemModel& systemModel);

        Eigen::Matrix<double, 6, 6> applyParticleFilter(Control& u);

    private:
        static Eigen::Matrix<double, 6, 1> stateToVector(PuckState& state);
    };
}

#endif //SRC_PARTICLEFILTER_HPP
