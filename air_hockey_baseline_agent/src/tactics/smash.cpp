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

using namespace air_hockey_baseline_agent;
using namespace Eigen;

Smash::Smash(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator), gen(rd()), dist(0, 2) {

}

bool Smash::ready() {
	return state.restart || ros::Time::now() > state.trajStopTime;
}

bool Smash::apply() {
	Vector3d xCur;
	generator.kinematics->forwardKinematics(state.observation.jointPosition,
			xCur);
	generator.transformations->applyInverseTransform(xCur);
	Vector2d xCur2d = xCur.block<2, 1>(0, 0);
	Vector2d puckCur2d = state.puckPredictedState.state.block<2, 1>(0, 0);

	Vector2d xGoal = computeTarget(puckCur2d);
	Vector2d vHit = (xGoal - puckCur2d).normalized();
	Vector2d xHit = puckCur2d
			- vHit * (envParams.puckRadius + envParams.malletRadius + 0.005);

	vHit *= agentParams.vHitMax;
	bool success = false;

	for (size_t i = 0; i < 10; ++i) {
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();

		bool ok = generator.combinatorialHit->plan(xCur2d, xHit, vHit,
				state.cartTrajectory);
		if (!ok) {
			return false;
		}
		generator.transformations->transformTrajectory(state.cartTrajectory);
		if (generator.optimizer->optimizeJointTrajectory(state.cartTrajectory,
				state.observation.jointPosition, state.jointTrajectory)) {
			success = true;
			break;
		} else {
			vHit *= .8;
			ROS_INFO_STREAM(
					"Optimization Failed [HITTING]. Reduce the velocity: " << vHit.transpose());
			continue;
		}
	}
	//! plan for return trajectory
	trajectory_msgs::MultiDOFJointTrajectory cartTrajReturn;
	trajectory_msgs::JointTrajectory jointTrajReturn;
	double vMax = 0.5;
	success = success && planReturnTraj(vMax, cartTrajReturn, jointTrajReturn);

	//! append return to whole trajectory
	if (success) {
		state.cartTrajectory.points.insert(state.cartTrajectory.points.end(),
				cartTrajReturn.points.begin(), cartTrajReturn.points.end());
		state.jointTrajectory.points.insert(state.jointTrajectory.points.end(),
				jointTrajReturn.points.begin(), jointTrajReturn.points.end());
		state.jointTrajectory.header.stamp = ros::Time::now();
		state.trajStopTime = ros::Time::now()
				+ ros::Duration(
						state.jointTrajectory.points.back().time_from_start);
		return true;
	} else {
		ROS_INFO_STREAM("Failed to find a feasible movement [HITTING]");
		return false;
	}
}

Vector2d Smash::computeTarget(Vector2d puckPosition) {
	Vector2d xTarget;
	auto random_integer = dist(gen);
	random_integer = 0;
	if (puckPosition(1) > 0.1) {
		xTarget.x() = envParams.tableLength;
		xTarget.y() = -0.1;
	} else if (puckPosition(1) < -0.1) {
		xTarget.x() = envParams.tableLength;
		xTarget.y() = 0.1;
	} else {
		xTarget.x() = envParams.tableLength;
		xTarget.y() = 0.0;
	}
	if (random_integer == 1) {
		ROS_INFO_STREAM("Strategy: Right");
		xTarget.y() = -2 * (envParams.tableWidth / 2 - envParams.puckRadius)
				- agentParams.xGoal.y();
	} else if (random_integer == 2) {
		ROS_INFO_STREAM("Strategy: Left");
		xTarget.y() = 2 * (envParams.tableWidth / 2 - envParams.puckRadius)
				- agentParams.xGoal.y();
	} else {
		ROS_INFO_STREAM("Strategy: Middle");
	}

	return xTarget;
}

Smash::~Smash() {

}
