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
#include <Eigen/Dense>

using namespace Eigen;
using namespace air_hockey_baseline_agent;

Cut::Cut(EnvironmentParams &envParams, AgentParams &agentParams,
         SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {
	collisionNumPrev = 0;
	waitForSteps = 0;
}

bool Cut::ready() {
	calculateCutPosition();
	if (collisionNumPrev != state.observation.puckPredictedState.numOfCollisions){
		collisionNumPrev = state.observation.puckPredictedState.numOfCollisions;
		waitForSteps = 5;
	} else if (state.observation.puckPredictedState.predictedTime > 0.3 &&
	     (xCut - xCutPrev).norm() > (envParams.puckRadius + envParams.malletRadius)) {
		if (differenceCount > 5){
			state.isNewTactics = true;
		} else {
			++differenceCount;
		}
	} else {
		differenceCount = 0;
	}

	return state.isNewTactics;
}

bool Cut::apply() {
	state.isNewTactics = false;

	xCutPrev = xCut;

	Vector3d xCur, vCur;
	Vector2d xCur2d, vCur2d;
	ros::Time tStart;
	JointArrayType qStart(agentParams.nq), dqStart(agentParams.nq);

	state.getPlannedJointState(qStart, dqStart, tStart,
	                           agentParams.planTimeOffset);

	generator.getCartesianPosAndVel(xCur, vCur, qStart, dqStart);

	generator.transformations->applyInverseTransform(xCur);
	generator.transformations->applyInverseRotation(vCur);

	xCur2d = xCur.block<2, 1>(0, 0);
	vCur2d = vCur.block<2, 1>(0, 0);
	double tStop = boost::algorithm::clamp(
			state.observation.puckPredictedState.predictedTime - 0.3,
			agentParams.tDefendMin, agentParams.tPredictionMax);

	xCut.x() = boost::algorithm::clamp(xCut.x(), envParams.malletRadius + 0.01, envParams.tableLength);
	xCut.y() = boost::algorithm::clamp(xCut.y(),
	                                   -envParams.tableWidth / 2 + envParams.malletRadius + 0.02,
	                                   envParams.tableWidth / 2 - envParams.malletRadius - 0.02);

	state.cartTrajectory.points.clear();
	state.jointTrajectory.points.clear();
	generator.cubicLinearMotion->plan(xCur2d, vCur2d, xCut,
	                                  Vector2d(0., 0.), tStop, state.cartTrajectory);
	generator.transformations->transformTrajectory(state.cartTrajectory);

	if (generator.optimizer->optimizeJointTrajectory(state.cartTrajectory, qStart,	state.jointTrajectory)) {
		state.jointTrajectory.header.stamp = tStart;
		state.cartTrajectory.header.stamp = tStart;
		return true;
	}

	ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Optimization Failed [Cut].");
	state.jointTrajectory.points.clear();
	state.cartTrajectory.points.clear();
	return false;
}

Cut::~Cut() {

}

void Cut::setNextState() {
	if (state.isPuckApproaching() && state.observation.puckPredictedState.predictedTime < agentParams.tPredictionMax &&
	    abs(state.observation.puckPredictedState.state.y()) > agentParams.defendZoneWidth / 2) {
		setTactic(CUT);
	} else {
		setTactic(READY);
		//! reset the cut record
		xCutPrev << 0., 0.;
	}
}

void Cut::calculateCutPosition() {
	if (waitForSteps > 0){
		--waitForSteps;
		return;
	}
	if (state.observation.puckPredictedState.state.y() > 0) {
		Vector2d vIn = state.observation.puckPredictedState.state.block<2, 1>(2, 0).normalized();
		Vector2d vOut(0., 1.);
		Vector2d offsetDir = (vIn - vOut).normalized();
		xCut = state.observation.puckPredictedState.state.block<2, 1>(0, 0) +
		       offsetDir * (envParams.puckRadius + envParams.malletRadius);
	} else {
		Vector2d vIn = state.observation.puckPredictedState.state.block<2, 1>(2, 0).normalized();
		Vector2d vOut(0., -1.);
		Vector2d offsetDir = (vIn - vOut).normalized();
		xCut = state.observation.puckPredictedState.state.block<2, 1>(0, 0) +
		       offsetDir * (envParams.puckRadius + envParams.malletRadius);
	}
}

