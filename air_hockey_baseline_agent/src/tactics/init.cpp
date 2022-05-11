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
	return state.isNewTactics;
}

bool Init::apply() {
	state.isNewTactics = false;

    state.tStart = ros::Time::now() + ros::Duration(agentParams.planTimeOffset);
    generator.getPlannedJointState(state, state.tStart);
    state.trajectoryBuffer.getFree().jointTrajectory.points.clear();
    state.trajectoryBuffer.getFree().cartTrajectory.points.clear();

    generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, agentParams.xInit, Vector3d(0., 0., 0.),
        2.0, state.trajectoryBuffer.getFree().cartTrajectory);
    generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);

    if (generator.optimizer->optimizeJointTrajectoryAnchor(state.trajectoryBuffer.getFree().cartTrajectory, state.qPlan, state.dqPlan,
        agentParams.qInit, 1.0, state.trajectoryBuffer.getFree().jointTrajectory)){

        generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);

        state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = state.tStart;
        state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = state.tStart;
        state.tPlan = state.tStart;
        return true;
    } else {
        ROS_ERROR_STREAM("Unable to optimize the trajectory");
        return false;
    }
}

Init::~Init() {

}

void Init::setNextState() {
	setTactic(READY);
}