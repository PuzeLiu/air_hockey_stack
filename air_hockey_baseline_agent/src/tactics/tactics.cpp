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

#include "air_hockey_baseline_agent/tactics.h"

#include <ros/ros.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace air_hockey_baseline_agent;

Tactic::Tactic(EnvironmentParams &envParams, AgentParams &agentParams,
			   SystemState &state, TrajectoryGenerator *generator) :
	envParams(envParams), agentParams(agentParams), state(state), generator(
	*generator) {

}

bool Tactic::generateStopTrajectory() {
	// Detect the stop point
	state.tStart = ros::Time::now() + ros::Duration(agentParams.planTimeOffset * 1.5);
	generator.getPlannedJointState(state, state.tStart);
	Vector3d xStop = state.xPlan;

	// Detect the start point
	state.tStart = ros::Time::now() + ros::Duration(agentParams.planTimeOffset);
	generator.getPlannedJointState(state, state.tStart);

	state.trajectoryBuffer.getFree().cartTrajectory.points.clear();
	state.trajectoryBuffer.getFree().jointTrajectory.points.clear();

	generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, xStop,
									  Vector3d(0., 0., 0.), agentParams.planTimeOffset * 2,
									  state.trajectoryBuffer.getFree().cartTrajectory);
	generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);

	bool opt_success;
	opt_success = generator.optimizer->optimizeJointTrajectoryAnchor(
		state.trajectoryBuffer.getFree().cartTrajectory,
		state.qPlan, state.dqPlan, JointArrayType(), agentParams.planTimeOffset * 2,
		state.trajectoryBuffer.getFree().jointTrajectory);

	if (opt_success) {
		ROS_INFO_STREAM(state.planPrevPoint);
		ROS_INFO_STREAM(state.trajectoryBuffer.getFree().jointTrajectory.points[0]);
		generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);
		generator.synchronizeCartesianTrajectory(state.trajectoryBuffer.getFree().jointTrajectory,
												 state.trajectoryBuffer.getFree().cartTrajectory);
		state.tPlan = state.tStart;
		state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = state.tStart;
		state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = state.tStart;

		//! Set start time stamp for next tactics
		state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset * 2);
		ROS_DEBUG_STREAM(state.trajectoryBuffer.getFree().jointTrajectory.header.stamp);
		ROS_DEBUG_STREAM(state.trajectoryBuffer.getFree().jointTrajectory);
		return true;
	}
	state.tPlan = ros::Time::now();
	ROS_DEBUG_STREAM_NAMED(agentParams.name, "Failed to generate STOP trajectory");
	return false;
}

Tactic::~Tactic() {

}

void Tactic::updateTactic() {
	if (state.observation.gameStatus.status == START) {
		setNextState();
	} else if (state.observation.gameStatus.status == STOP) {
		setTactic(INIT);
	} else if (state.observation.gameStatus.status == PAUSE) {
		setTactic(INIT);
	}
}

void Tactic::setTactic(Tactics tactic) {
	if (tactic != state.currentTactic) {
		state.isNewTactics = true;
		state.tNewTactics = ros::Time::now().toSec() + agentParams.tTacticSwitchMin;
		ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name +
			": " + "Tactics changed: " << tactic2String(state.currentTactic) << " -> " << tactic2String(tactic));
		state.currentTactic = tactic;
	}
}

std::string Tactic::tactic2String(Tactics tactic) {
	switch (tactic) {
		case Tactics::INIT: return "INIT   ";
		case Tactics::SMASH: return "SMASH  ";
		case Tactics::CUT: return "CUT    ";
		case Tactics::READY: return "READY  ";
		case Tactics::REPEL: return "REPEL  ";
		case Tactics::PREPARE: return "PREPARE";
		default: ROS_ERROR_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Invalid Tactic");
			exit(-1);
	}
}

bool Tactic::canSmash() {
	if (state.observation.puckPredictedState.state.x() < agentParams.hitRange.mean() &&
		state.observation.puckPredictedState.state.x() > agentParams.hitRange[0] &&
		abs(state.observation.puckPredictedState.state.y()) < (envParams.tableWidth / 2 - envParams.puckRadius - 0.1)) {
		return true;
	}
	if (state.observation.puckPredictedState.state.x() > agentParams.hitRange.mean() &&
		state.observation.puckPredictedState.state.x() < agentParams.hitRange[1] &&
		state.observation.puckPredictedState.state.dx() < 0) {
		return true;
	}
	return false;
}

bool Tactic::shouldCut() {
	if (state.observation.puckPredictedState.predictedTime < agentParams.tPredictionMax &&
		(state.observation.puckPredictedState.numOfCollisions > 0 ||
			(abs(state.observation.puckPredictedState.state.y()) < (envParams.tableWidth / 2 - envParams.malletRadius))
		)) {
		return true;
	}
	return false;
}

bool Tactic::shouldRepel() {
	if ((state.observation.puckPredictedState.numOfCollisions == 0) &&
		(state.observation.puckPredictedState.predictedTime < agentParams.tPredictionMax &&
			state.observation.puckPredictedState.predictedTime > agentParams.tPredictionMax / 2 &&
			abs(state.observation.puckPredictedState.state.y()) < agentParams.defendZoneWidth / 2)) {
		return true;
	}
	return false;
}

bool Tactic::puckStuck() {
	if (state.isPuckStatic()) {
		if ((state.observation.puckPredictedState.state.x() < agentParams.hitRange[0] &&
			abs(state.observation.puckPredictedState.state.y()) <
				(envParams.tableWidth / 2 - 2 * envParams.puckRadius))) {
			return true;
		}
		if (state.observation.puckPredictedState.state.x() < agentParams.hitRange[1] &&
			state.observation.puckPredictedState.state.dx() < 0.05 &&
			abs(state.observation.puckPredictedState.state.y())
				> (envParams.tableWidth / 2 - envParams.puckRadius - 0.1)) {
			return true;
		}
	}
	return false;
}


