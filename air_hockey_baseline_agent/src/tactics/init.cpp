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
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace Eigen;
using namespace trajectory_msgs;
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

Init::Init(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Init::ready() {
    state.restart = ((state.observation.jointPosition - agentParams.qInit).norm() > 1e-3);
	return state.restart && ros::Time::now() > state.trajStopTime;
}

bool Init::apply() {
	JointTrajectoryPoint jointViaPoint_;
	jointViaPoint_.positions.resize(7);
	jointViaPoint_.velocities.resize(7);

	state.jointTrajectory.points.clear();
	for (int i = 0; i < 7; ++i) {
		jointViaPoint_.positions[i] = agentParams.qInit[i];
		jointViaPoint_.velocities[i] = 0.;
	}
	jointViaPoint_.time_from_start = ros::Duration(5.0);
	state.jointTrajectory.points.push_back(jointViaPoint_);

	state.jointTrajectory.header.stamp = ros::Time::now();
	state.trajStopTime = state.jointTrajectory.header.stamp
			+ ros::Duration(5.0);

	ROS_INFO_STREAM("Go to initial position");

	return true;
}

Init::~Init() {

}
