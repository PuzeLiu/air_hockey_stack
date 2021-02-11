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
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

Prepare::Prepare(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Prepare::ready() {
	return state.restart || ros::Time::now() > state.trajStopTime_;
}

bool Prepare::apply() {
	Vector3d xStart, vStart;
	Vector2d xStart2d, vStart2d, xPuck, xPrepare, vPrepare;
	ros::Time tStart;
	Kinematics::JointArrayType qStart, dqStart;

	state.getPlannedState(xStart, vStart, qStart, dqStart, tStart,
			planTimeOffset_);
	generator.transformations->applyInverseTransform(xStart);
	generator.transformations->applyInverseRotation(vStart);

	xStart2d = xStart.block<2, 1>(0, 0);

	xPuck = observationState_.puckPredictedState.state.block<2, 1>(0, 0);
	if (xPuck.y() > 0) {
		vPrepare =
				(Vector2d(xPuck.x() + 0.2,
						2 * (envParams.tableWidth_ / 2 - envParams.puckRadius_))
						- xPuck).normalized();
//            vPrepare = Vector2d(0.1, 0.7);
	} else {
		vPrepare = (Vector2d(xPuck.x() + 0.2,
				-2 * (envParams.tableWidth_ / 2 - envParams.puckRadius_))
				- xPuck).normalized();
//            vPrepare = Vector2d(0.1, 0.7);
	}
	xPrepare = xPuck
			- vPrepare * (envParams.puckRadius_ + envParams.malletRadius_);
	xPrepare.x() = boost::algorithm::clamp(xPrepare.x(),
			envParams.malletRadius_ + 0.02,
			envParams.tableLength_ - envParams.malletRadius_ - 0.02);
	xPrepare.y() = boost::algorithm::clamp(xPrepare.y(),
			-envParams.tableWidth_ / 2 + envParams.malletRadius_ + 0.02,
			envParams.tableWidth_ / 2 - envParams.malletRadius_ - 0.02);
	double tStop = 0.08;
	bool success = false;

	for (int i = 0; i < 10; ++i) {
		state.cartTrajectory_.points.clear();
		state.jointTrajectory_.points.clear();
		generator.combinatorialHit->plan(xStart2d, xPrepare, vPrepare,
				state.cartTrajectory_, tStop);
		generator.transformations->transformTrajectory(state.cartTrajectory_);

		success = generator.optimizer->optimizeJointTrajectory(
				state.cartTrajectory_, observationState_.jointPosition,
				state.jointTrajectory_);
		if (success) {
			break;
		}

		vPrepare *= .8;
		ROS_INFO_STREAM(
				"Optimization Failed [PREPARE]. Reduce the velocity: " << vPrepare.transpose());
	}

	//! plan for return trajectory
	trajectory_msgs::MultiDOFJointTrajectory cartTrajReturn;
	trajectory_msgs::JointTrajectory jointTrajReturn;
	double vMax = 0.5;
	success = success
			&& planReturnTraj(state, vMax, cartTrajReturn, jointTrajReturn);

	//! append return to whole trajectory
	if (success) {
		state.cartTrajectory_.points.insert(state.cartTrajectory_.points.end(),
				cartTrajReturn.points.begin(), cartTrajReturn.points.end());
		state.jointTrajectory_.points.insert(
				state.jointTrajectory_.points.end(),
				jointTrajReturn.points.begin(), jointTrajReturn.points.end());
		state.cartTrajectory_.header.stamp = tStart;
		state.jointTrajectory_.header.stamp = tStart;

		state.trajStopTime_ = ros::Time::now()
				+ ros::Duration(
						state.jointTrajectory_.points.back().time_from_start);
		return true;
	} else {
		ROS_INFO_STREAM(
				"Optimization Failed [PREPARE]. Failed to find a feasible hitting movement");
		return false;
	}
}

Prepare::~Prepare() {

}

