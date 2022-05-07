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
		double last_point_time;
		if (!state.jointTrajectory.points.empty()){
			last_point_time = state.jointTrajectory.points.back().time_from_start.toSec();
		}
		state.tStart = state.tPlan + ros::Duration(std::min(agentParams.planTimeOffset, last_point_time));
	} else {
		state.tStart = state.tPlan + ros::Duration(2.0);
	}
	generator.getPlannedJointState(state, state.tStart);
	return true;
}

bool Ready::apply() {
	state.isNewTactics = false;
	if (ros::Time::now() >= state.tPlan)
	{
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();

		double tStop = std::max((agentParams.xHome - state.xPlan).norm() / 1.0, 2.0);

		generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, agentParams.xHome, Vector3d(0., 0., 0.),
			tStop, state.cartTrajectory);

		generator.transformations->transformTrajectory(state.cartTrajectory);

		if (generator.optimizer->optimizeJointTrajectoryAnchor(state.cartTrajectory, state.qPlan,
			agentParams.qHome, state.cartTrajectory.points.back().time_from_start.toSec() / 2, state.jointTrajectory)) {
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

Ready::~Ready() {

}

void Ready::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
        if (!agentParams.debugTactics) {
            if(state.isPuckStatic()){
                if (canSmash()) {
                    setTactic(SMASH);
                } else if (puckStuck()) {
                    setTactic(PREPARE);
                }
            } else if(state.isPuckApproaching()){
                if(shouldRepel()){
                    setTactic(REPEL);
                }else if (shouldCut()){
                    setTactic(CUT);
                }
            }else {
                setTactic(READY);
            }
        }
	}
}



