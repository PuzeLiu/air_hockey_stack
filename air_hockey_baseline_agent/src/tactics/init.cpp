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
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace Eigen;
using namespace trajectory_msgs;
using namespace air_hockey_baseline_agent;

Init::Init(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Init::ready() {
	if (state.isNewTactics) {
		state.tPlan = ros::Time::now();
		state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset);
		return true;
	} else if (not hasPlannedToInitPoint) {
		return true;
	} else {
		return false;
	}
}

bool Init::apply() {
	if (state.isNewTactics) {
		state.isNewTactics = false;
		hasPlannedToInitPoint = false;
		generator.getPlannedJointState(state, state.tStart);
		if (state.vPlan.norm() > 0.01) {
			return generateStopTrajectory();
		}
	}

	if (ros::Time::now() >= state.tPlan) {
		generator.getPlannedJointState(state, state.tStart);
		return planTrajectory();
	}

	return false;
}

Init::~Init() {

}

void Init::setNextState() {
	setTactic(READY);
}

bool Init::planTrajectory() {
	state.trajectoryBuffer.getFree().jointTrajectory.points.clear();
	state.trajectoryBuffer.getFree().cartTrajectory.points.clear();

	generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, agentParams.xInit, Vector3d(0., 0., 0.),
									  2.0, state.trajectoryBuffer.getFree().cartTrajectory);
	generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);

	if (generator.optimizer->optimizeJointTrajectoryAnchor(state.trajectoryBuffer.getFree().cartTrajectory,
														   state.qPlan, state.dqPlan,
														   agentParams.qInit, 1.0,
														   state.trajectoryBuffer.getFree().jointTrajectory)){
		generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);
		generator.synchronizeCartesianTrajectory(state.trajectoryBuffer.getFree().jointTrajectory,
												 state.trajectoryBuffer.getFree().cartTrajectory);
		state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = state.tStart;
		state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = state.tStart;
		state.tPlan = state.tStart + state.trajectoryBuffer.getFree().jointTrajectory.points.back().time_from_start;
		hasPlannedToInitPoint = true;
		return true;
	}
	state.tPlan = ros::Time::now() + ros::Duration(agentParams.planTimeOffset);
	state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
	state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
	ROS_INFO_STREAM_NAMED(agentParams.name,
						  agentParams.name + ": " + "Failed to find a feasible movement [INIT]");
	return false;
}