#include "air_hockey_puck_tracker/EKF_Wrapper.hpp"

using namespace air_hockey_baseline_agent;

const PuckState & EKF_Wrapper::update(ObservationModel &m, const Measurement &z) {
    m.updateJacobians(x);

    // COMPUTE KALMAN GAIN
    // compute innovation covariance
    S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());

    // compute kalman gain
    KalmanGain <Measurement> K = P * m.H.transpose() * S.inverse();

    // UPDATE STATE ESTIMATE AND COVARIANCE
    // Update state using computed kalman gain and innovation
    calculateInnovation(m, z);
    x += K * (mu);

    // Update covariance
    P -= K * m.H * P;

    // return updated state estimate
    return x;
}

const air_hockey_baseline_agent::EKF_Wrapper::InnovationCovariance &
air_hockey_baseline_agent::EKF_Wrapper::updateInnovationCovariance(ObservationModel &m) {
    S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());
    return S;
}

void air_hockey_baseline_agent::EKF_Wrapper::calculateInnovation(ObservationModel &m, const Measurement &z) {
    mu = z - m.h(x);
    Eigen::Rotation2Dd rot(mu.theta());
    mu.theta() = rot.smallestAngle();
}

void air_hockey_baseline_agent::EKF_Wrapper::moveOneStep(SystemModel &s, const Control &u) {
	x = s.f(x, u);
}