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
using namespace air_hockey_baseline_agent;

Home::Home(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState *state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {
	Vector3d xTmp, gc;
	Quaterniond quatTmp;
	double psi;

	auto kinematics = this->generator.kinematics;
	auto transformations = this->generator.transformations;
	auto optimizer = this->generator.optimizer;

	kinematics->forwardKinematics(agentParams.qRef_, xTmp, quatTmp);
	kinematics->getRedundancy(agentParams.qRef_, gc, psi);

	xTmp << agentParams.xHome_[0], agentParams.xHome_[1], envParams.universalJointHeight_;
	transformations->applyForwardTransform(xTmp);
	if (!kinematics->inverseKinematics(xTmp, quatTmp, gc, psi, qHome)) {
		ROS_ERROR_STREAM(
				"Inverse Kinematics fail, unable to find solution for HOME position");
	}

	optimizer->SolveJoint7(qHome);
}

bool Home::ready() {
	return state.restart || ros::Time::now() > state.trajStopTime_;
}

bool Home::apply() {
	JointTrajectoryPoint jointViaPoint_;
	jointViaPoint_.positions.resize(7);
	jointViaPoint_.velocities.resize(7);

	state.jointTrajectory_.points.clear();
	for (int i = 0; i < 7; ++i) {
		jointViaPoint_.positions[i] = qHome[i];
		jointViaPoint_.velocities[i] = 0.;
	}

	jointViaPoint_.time_from_start = ros::Duration(2.0);
	state.jointTrajectory_.points.push_back(jointViaPoint_);

	state.jointTrajectory_.header.stamp = ros::Time::now();
	state.trajStopTime_ = state.jointTrajectory_.header.stamp
			+ ros::Duration(2.0);

	return true;
}

Home::~Home() {

}

