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
	if (state.isNewTactics) {
		// Get planned position and velocity at now + timeoffset
		state.tPlan = ros::Time::now();
		double last_point_time;
		if (!state.jointTrajectory.points.empty()){
			last_point_time = state.jointTrajectory.points.back().time_from_start.toSec();
		}
		state.tStart = state.tPlan + ros::Duration(std::min(agentParams.planTimeOffset, last_point_time));
	} else {
		state.tStart = state.tPlan + ros::Duration(agentParams.defendPlanSteps / agentParams.rate);
	}
	generator.getPlannedJointState(state, state.tStart);
	calculateCutPosition();
	return true;
}

bool Cut::apply() {
	double tStop;
	state.isNewTactics = false;
	if (ros::Time::now() > state.tPlan)
	{
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();

		tStop = std::max(agentParams.defendPlanSteps / agentParams.rate,
			(xCut - state.xPlan).norm() / agentParams.defendMaxEEVelocity);

		tStop = std::max(tStop, state.observation.puckPredictedState.predictedTime - 0.5);

		generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, xCut, Vector3d(0., 0., 0.),
				tStop, agentParams.defendPlanSteps, state.cartTrajectory);

		generator.transformations->transformTrajectory(state.cartTrajectory);

		if (generator.optimizer->optimizeJointTrajectory(state.cartTrajectory, state.qPlan, state.jointTrajectory))
		{
			if (state.jointTrajectory.points.front().time_from_start.toSec() == 0) {
				for (int j = 0; j < agentParams.nq; ++j)
				{
					state.jointTrajectory.points.front().positions[j] = state.qPlan[j];
					state.jointTrajectory.points.front().velocities[j] = state.dqPlan[j];
				}
			}
			generator.cubicSplineInterpolation(state.jointTrajectory);
			state.tPlan = state.tStart;
			state.jointTrajectory.header.stamp = state.tStart;
			state.cartTrajectory.header.stamp = state.tStart;
			return true;
		}
		else
		{
			ROS_INFO_STREAM("Plan Failed");
			return false;
		}
	}
	return false;
}

Cut::~Cut() {

}

void Cut::setNextState() {
	if (not state.isPuckStatic() and (not state.isPuckApproaching() or not shouldCut())) {
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
	Vector2d cutTmp;
	if (state.observation.puckPredictedState.state.y() > 0) {
		Vector2d vIn = state.observation.puckPredictedState.state.block<2, 1>(2, 0).normalized();
		Vector2d vOut(0., 1.);
		Vector2d offsetDir = (vIn - vOut).normalized();
		cutTmp = state.observation.puckPredictedState.state.block<2, 1>(0, 0) +
		       offsetDir * (envParams.puckRadius + envParams.malletRadius);
	} else {
		Vector2d vIn = state.observation.puckPredictedState.state.block<2, 1>(2, 0).normalized();
		Vector2d vOut(0., -1.);
		Vector2d offsetDir = (vIn - vOut).normalized();
		cutTmp = state.observation.puckPredictedState.state.block<2, 1>(0, 0) +
		       offsetDir * (envParams.puckRadius + envParams.malletRadius);
	}

	if (state.isNewTactics) {
		xCut.topRows(2) = cutTmp;
	} else {
		xCut.topRows(2) = (1 - agentParams.defendTargetUpdateRatio) * xCut.topRows(2) +
			agentParams.defendTargetUpdateRatio * cutTmp;
	}
	xCut[2] = agentParams.universalJointHeight;
}

