//
// Created by default on 26.05.21.
//

#include <random>
#include "ros/ros.h"
#include "../include/air_hockey_puck_tracker/ParticleFilter.hpp"
#include <tf2/LinearMath/Scalar.h>

using namespace air_hockey_baseline_agent;
using namespace std;

ParticleFilter::ParticleFilter(SystemModel& systemModel): systemModel_(systemModel){

}

void ParticleFilter::sampleParticles(PuckState &state, Eigen::Matrix<double, 6,6> &covariance) {

    Eigen::VectorXd mean = stateToVector(state);
    Eigen::MatrixXd normTransform(6,6);

    Eigen::LLT<Eigen::MatrixXd> cholSolver(covariance);

    if (cholSolver.info()==Eigen::Success) {
        // Use cholesky solver
        normTransform = cholSolver.matrixL();
    } else {
        ROS_WARN_STREAM("[Particle Filter] Cholesky method has failed, the matrix is not a valid covariance matrix");
    }

    // delete particles from last sample
    particles.clear();

    for (int i = 0; i < sampleSize; i++) {
        Eigen::VectorXd sample = mean + normTransform * Eigen::VectorXd::Random(6);
        PuckState sampleState{sample};
        particles.push_back(sampleState);
    }

}

Eigen::Matrix<double, 6 , 6> ParticleFilter::applyParticleFilter(Control& u) {
    std::vector<PuckState> predictedNextState;
    for (auto &particle : particles) {
        predictedNextState.push_back(systemModel_.f(particle, u));
    }

    //calculate mean of next step
    Eigen::Matrix<double, 6, 1> mean;
    mean.setZero();
    for(auto &nextState: predictedNextState) {
        mean += stateToVector(nextState);
    }


    //normalize angle theta
    mean(2,0) = tf2NormalizeAngle(mean(2,0));

    mean = (1.0/predictedNextState.size()) * mean;

    // calculate new covariance
    Eigen::Matrix<double, 6, 6> newCovariance = Eigen::Matrix<double, 6, 6>::Zero();

    for (auto &state: predictedNextState){
        Eigen::Matrix<double, 6, 1> diff = stateToVector(state) - mean;
        Eigen::Matrix<double, 1,6> tdiff = diff.transpose();
        Eigen::Matrix<double, 6, 6> e = diff * tdiff;
        newCovariance += e;
    }
    return (1/(predictedNextState.size() -1)) * newCovariance;
}

Eigen::Matrix<double, 6, 1> ParticleFilter::stateToVector(PuckState &state){
    return Eigen::Matrix<double, 6, 1>{state.x(), state.y(), state.dx(), state.dy(), state.theta(), state.dtheta()};
}


std::vector<PuckState> ParticleFilter::getParticles(){
    return particles;
}