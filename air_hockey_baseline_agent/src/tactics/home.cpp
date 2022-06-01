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

Home::Home(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Home::ready() {
    if (state.isNewTactics) {
        state.tPlan = ros::Time::now();
        state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset);
    }
	return true;
}

bool Home::apply() {
	state.isNewTactics = false;

    if (ros::Time::now() >= state.tPlan) {
        generator.getPlannedJointState(state, state.tStart);
        state.jointTrajectory.points.clear();
        state.cartTrajectory.points.clear();

        generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, agentParams.xHome, Vector3d(0., 0., 0.),
            2.0, state.cartTrajectory);
        generator.transformations->transformTrajectory(state.cartTrajectory);

        if (!generator.optimizer->optimizeJointTrajectoryAnchor(state.cartTrajectory, state.qPlan, agentParams.qHome, 1.0, state.jointTrajectory)){
            state.jointTrajectory.header.stamp = state.tStart;
            state.cartTrajectory.header.stamp = state.tStart;
            state.tPlan = state.tStart;
            return true;
        } else {
            state.tPlan = ros::Time::now();
            return false;
        }
    }
    return false;
}

Home::~Home() {

}

void Home::setNextState() {
	if (state.hasActiveTrajectory()){
		setTactic(HOME);
	} else {
		setTactic(READY);
	}
}
