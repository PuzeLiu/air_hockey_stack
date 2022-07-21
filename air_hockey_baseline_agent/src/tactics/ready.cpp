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
#include <ros/ros.h>

using namespace Eigen;
using namespace air_hockey_baseline_agent;

Ready::Ready(EnvironmentParams &envParams, AgentParams &agentParams,
			 SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {
	debugCount = 0;
}

bool Ready::ready() {
	//! New cut implementation
	if (state.isNewTactics) {
		// Get planned position and velocity at now + timeoffset
		state.tPlan = ros::Time::now();
		state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset);
	} else {
		state.tStart = state.tPlan + ros::Duration(1.0);
		if (state.tStart <= ros::Time::now() + ros::Duration(agentParams.planTimeOffset)) {
			state.tStart = ros::Time::now() + ros::Duration(agentParams.planTimeOffset);
		}
	}
	return true;
}

bool Ready::apply() {
	state.isNewTactics = false;
	if (ros::Time::now() >= state.tPlan) {
		generator.getPlannedJointState(state, state.tStart);
		state.trajectoryBuffer.getFree().cartTrajectory.points.clear();
		state.trajectoryBuffer.getFree().jointTrajectory.points.clear();

		double tStop = std::max((agentParams.xHome - state.xPlan).norm() / 1.0, 1.0);

		generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, agentParams.xHome, Vector3d(0., 0., 0.),
										  tStop, state.trajectoryBuffer.getFree().cartTrajectory);

		generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);
		if (generator.optimizer->optimizeJointTrajectoryAnchor(state.trajectoryBuffer.getFree().cartTrajectory,
															   state.qPlan, state.dqPlan,
															   agentParams.qHome, tStop / 2,
															   state.trajectoryBuffer.getFree().jointTrajectory)) {
			generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);
			generator.synchronizeCartesianTrajectory(state.trajectoryBuffer.getFree().jointTrajectory,
													 state.trajectoryBuffer.getFree().cartTrajectory);

			state.tPlan = state.tStart;
			state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = state.tStart;
			state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = state.tStart;
			return true;
		} else {
			ROS_INFO_STREAM("Plan Failed");
			return false;
		}
	}
	return false;
}

Ready::~Ready() {

}

void Ready::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
		if (!agentParams.debugTactics) {
			if (state.isPuckStatic()) {
				if (canSmash()) {
					setTactic(SMASH);
				} else if (puckStuck()) {
					setTactic(PREPARE);
				}
			} else if (state.isPuckApproaching()) {
				if (shouldRepel()) {
					setTactic(REPEL);
				} else if (shouldCut()) {
					setTactic(CUT);
				}
			} else {
				setTactic(READY);
			}
		} else {
			if (agentParams.debuggingTactic == SMASH) {
				setTactic(Tactics::SMASH);
				agentParams.debuggingTactic = READY;
			} else if (agentParams.debuggingTactic == CUT) {
				if (state.isPuckApproaching() && shouldCut()) {
					setTactic(Tactics::CUT);
				}
			} else if (agentParams.debuggingTactic == REPEL) {
				if (state.isPuckApproaching() && shouldRepel()) {
					setTactic(Tactics::REPEL);
					agentParams.debuggingTactic = READY;
				}
			} else if (agentParams.debuggingTactic == PREPARE) {
				setTactic(Tactics::PREPARE);
				agentParams.debuggingTactic = READY;
			} else if (agentParams.debugTactics == READY) {
			}
		}
	}
}



