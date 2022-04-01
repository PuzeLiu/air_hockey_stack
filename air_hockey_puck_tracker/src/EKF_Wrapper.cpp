#include "air_hockey_puck_tracker/EKF_Wrapper.hpp"

using namespace air_hockey_baseline_agent;

EKF_Wrapper::EKF_Wrapper()
{
	checkMallet = false;
}

const PuckState& EKF_Wrapper::predict(SystemModel& s, const Control& u)
{
	// check collision
	hasCollision = s.collisionModel->applyCollision(x, checkMallet, s.F);
	if (!hasCollision){
		s.updateJacobians(x, u);

		// predict state
		x = s.f(x, u);
	}

	// predict covariance
	P = (s.F * P * s.F.transpose()) + (s.W * s.getCovariance() * s.W.transpose());

	// return state prediction
	return this->getState();
}

const PuckState& EKF_Wrapper::update(ObservationModel& m, const Measurement& z)
{
	m.updateJacobians(x);

	// COMPUTE KALMAN GAIN
	// compute innovation covariance
	updateInnovationCovariance(m);

	// compute kalman gain
	K = P * m.H.transpose() * innovationCovariance.inverse();

	// UPDATE STATE ESTIMATE AND COVARIANCE
	// Update state using computed kalman gain and innovation
	calculateInnovation(m, z);
	x += K * (innovation);

	// Update covariance
	P -= K * m.H * P;

	// return updated state estimate
	return x;
}

const EKF_Wrapper::InnovationCovarianceType& EKF_Wrapper::updateInnovationCovariance(ObservationModel& m)
{
	innovationCovariance = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());
	return innovationCovariance;
}

void EKF_Wrapper::calculateInnovation(ObservationModel& m, const Measurement& z)
{
	innovation = z - m.h(x);
	Eigen::Rotation2Dd rot(innovation.theta());
	innovation.theta() = rot.smallestAngle();
}

void EKF_Wrapper::moveOneStep(SystemModel& s, const Control& u)
{
	hasCollision = s.collisionModel->applyCollision(x, checkMallet, s.F);
	if (not hasCollision){
		x = s.f(x, u);
	}
}

void EKF_Wrapper::isPuckOutsideBoundary(const SystemModel& systemModel, const Measurement& m)
{
	isStateInsideTable = !systemModel.isOutsideBoundary(x);
	isMeasurementInsideTable = !systemModel.isOutsideBoundary(m);
}