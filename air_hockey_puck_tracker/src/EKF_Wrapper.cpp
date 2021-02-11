#include "air_hockey_puck_tracker/EKF_Wrapper.hpp"

using namespace air_hockey_baseline_agent;

const State & EKF_Wrapper::update(ObservationModel &m, const Measurement &z) {
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
air_hockey_baseline_agent::EKF_Wrapper::updateInnovationCovariance(air_hockey_baseline_agent::ObservationModel &m) {
    S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());
    return S;
}

void air_hockey_baseline_agent::EKF_Wrapper::calculateInnovation(air_hockey_baseline_agent::ObservationModel &m, const air_hockey_baseline_agent::Measurement &z) {
    mu = z - m.h(x);
    Eigen::Rotation2Dd rot(mu.theta());
    mu.theta() = rot.smallestAngle();
}
