/*
 * MIT License
 * Copyright (c) 2020-2021 Puze Liu, Davide Tateo
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

#include "air_hockey_baseline_agent/system_state.h"

using namespace std;
using namespace air_hockey_baseline_agent;

SystemState::SystemState() {
}

SystemState::~SystemState() {
	delete observer;
}

void SystemState::init(ros::NodeHandle nh, const AgentParams &agentParams, std::string controllerName)
{
    trajectoryBuffer = TrajectoryBuffer(agentParams);

	currentTactic = Tactics::INIT;

	isNewTactics = false;

	staticCount = 0;
	approachingCount = 0;

	qPlan.resize(agentParams.nq);
	dqPlan.resize(agentParams.nq);
    planPrevPoint.positions.resize(agentParams.nq);
    planPrevPoint.velocities.resize(agentParams.nq);

	observation.jointPosition.resize(agentParams.nq);
	observation.jointVelocity.resize(agentParams.nq);
	observation.jointDesiredPosition.resize(agentParams.nq);
	observation.jointDesiredVelocity.resize(agentParams.nq);
	observation.jointPosition.setZero();
	observation.jointVelocity.setZero();
	observation.jointDesiredPosition.setZero();
	observation.jointDesiredVelocity.setZero();
	observation.puckEstimatedState.setZero();
	observation.puckPredictedState.state.setZero();
	observation.puckPredictedState.covariance.setZero();

	observer = new Observer(nh, controllerName, &observation, agentParams.defendLine);
}

bool SystemState::hasActiveTrajectory() {
    return ros::Time::now() <= trajectoryBuffer.getExec().jointTrajectory.header.stamp +
    trajectoryBuffer.getExec().jointTrajectory.points.back().time_from_start;
}

bool SystemState::isPuckStatic() {
	return staticCount > 10;
}

bool SystemState::isPuckApproaching() {
	return approachingCount > 10;
}

void SystemState::updateObservationAndState(const AgentParams& agentParams){
	observer->updateObservation();
	if (observation.puckPredictedState.state.block<2, 1>(2, 0).norm() < agentParams.staticVelocityThreshold){
		++staticCount;
	} else {
		staticCount = 0;
	}

	if (observation.puckEstimatedState.dx() < 0){
		++approachingCount;
	} else {
		approachingCount = 0;
	}

	staticCount = min(staticCount, 20);
	approachingCount = min(approachingCount, 20);

    if (ros::Time::now() >= trajectoryBuffer.getFree().jointTrajectory.header.stamp and
    trajectoryBuffer.getFree().jointTrajectory.header.stamp > trajectoryBuffer.getExec().jointTrajectory.header.stamp) {
        trajectoryBuffer.moveNext();
    }
}
