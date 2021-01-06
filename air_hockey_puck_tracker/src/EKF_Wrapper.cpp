#include "air_hockey_puck_tracker/EKF_Wrapper.hpp"

using namespace AirHockey;

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

const AirHockey::EKF_Wrapper::InnovationCovariance &
AirHockey::EKF_Wrapper::updateInnovationCovariance(AirHockey::ObservationModel &m) {
    S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());
    return S;
}

void AirHockey::EKF_Wrapper::calculateInnovation(AirHockey::ObservationModel &m, const AirHockey::Measurement &z) {
    mu = z - m.h(x);
    Eigen::Rotation2Dd rot(mu.theta());
    mu.theta() = rot.smallestAngle();
}
