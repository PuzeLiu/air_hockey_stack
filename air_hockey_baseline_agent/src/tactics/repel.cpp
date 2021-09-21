/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
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

using namespace Eigen;
using namespace air_hockey_baseline_agent;

Repel::Repel(EnvironmentParams &envParams, AgentParams &agentParams,
         SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {
	counter = 0;
}

bool Repel::ready() {
	if (state.isNewTactics && counter < 15){
		++counter;
		return false;
	} else if (state.isNewTactics && counter >= 15){
		return true;
	} else return state.isNewTactics;
}

bool Repel::apply() {
	JointArrayType qStart(agentParams.nq), dqStart(agentParams.nq);
	ros::Time tStart;
	state.getPlannedJointState(qStart, dqStart, tStart, agentParams.planTimeOffset);

	if (dqStart.norm() > 0.05){
		generateStopTrajectory();
		return true;
	} else {
		state.isNewTactics = false;
		return generateRepelTrajectory(qStart, tStart);
	}
}

Repel::~Repel() {

}

void Repel::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
		if (state.hasActiveTrajectory()) {
			setTactic(REPEL);
		} else {
			counter = 0;
			setTactic(READY);
		}
	}
}

bool Repel::generateRepelTrajectory(const JointArrayType &qCur, ros::Time &tStart) {
	Vector2d xCur2d, xHit2d, vHit2d;
	Vector3d xCur;

	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, qCur);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	xCur = generator.agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();
	generator.transformations->applyInverseTransform(xCur);
	xCur2d = xCur.block<2, 1>(0, 0);

	xHit2d = state.observation.puckPredictedState.state.block<2, 1>(0, 0);
	vHit2d = state.observation.puckPredictedState.state.block<2, 1>(2, 0).normalized();
	vHit2d = -vHit2d * 1.2;

	for (size_t i = 0; i < 10; ++i) {
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();

		generator.combinatorialHit->plan(xCur2d, xHit2d, vHit2d,state.cartTrajectory);
		generator.transformations->transformTrajectory(state.cartTrajectory);

		if (generator.optimizer->optimizeJointTrajectory(state.cartTrajectory,
			state.observation.jointPosition, state.jointTrajectory)) {
			auto tHitStart = state.observation.stamp +
						     ros::Duration(state.observation.puckPredictedState.predictedTime) -
			                 state.jointTrajectory.points.back().time_from_start;
			if (ros::Time::now() <= tHitStart){
				tStart = tHitStart;
				state.jointTrajectory.header.stamp = tStart;
				state.cartTrajectory.header.stamp = tStart;
				return true;
			} else {
				state.jointTrajectory.points.clear();
				state.cartTrajectory.points.clear();
				return false;
			}
		} else {
			vHit2d *= .9;
			ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
								  "Optimization Failed [REPEL]. Reduce the velocity: " << vHit2d.transpose());
			continue;
		}
	}


	return false;
}