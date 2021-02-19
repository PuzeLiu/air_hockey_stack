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
using namespace trajectory_msgs;
using namespace air_hockey_baseline_agent;

Prepare::Prepare(EnvironmentParams &envParams, AgentParams &agentParams,
                 SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Prepare::ready() {
	return state.isNewTactics;
}

bool Prepare::apply() {
	iiwas_kinematics::Kinematics::JointArrayType qStart, dqStart;
	ros::Time tStart;
	state.getPlannedJointState(qStart, dqStart, tStart, agentParams.planTimeOffset);

	if (dqStart.norm() > 0.05) {
		generateStopTrajectory();
		return true;
	} else {
		state.isNewTactics = false;
		return generatePrepareTrajectory(qStart, dqStart, tStart);
	}
}

Prepare::~Prepare() {

}

void Prepare::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
		if (state.hasActiveTrajectory()) {
			setTactic(PREPARE);
		} else {
			setTactic(READY);
		}
	}
}

bool Prepare::generatePrepareTrajectory(iiwas_kinematics::Kinematics::JointArrayType &qStart,
                                        iiwas_kinematics::Kinematics::JointArrayType &dqStart,
                                        ros::Time tStart) {
	Vector3d xStart, vStart;
	Vector2d xStart2d, vStart2d, xPuck, xPrepare, vPrepare;

	generator.getCartesianPosAndVel(xStart, vStart, qStart, dqStart);

	generator.transformations->applyInverseTransform(xStart);
	generator.transformations->applyInverseRotation(vStart);

	xStart2d = xStart.block<2, 1>(0, 0);

	xPuck = state.observation.puckPredictedState.state.block<2, 1>(0, 0);
	if (xPuck.y() > 0) {
		vPrepare = (Vector2d(xPuck.x() + 0.2, 2 * (envParams.tableWidth / 2 - envParams.puckRadius))
		            - xPuck).normalized();
	} else {
		vPrepare = (Vector2d(xPuck.x() + 0.2, -2 * (envParams.tableWidth / 2 - envParams.puckRadius))
		            - xPuck).normalized();
	}
	xPrepare = xPuck - vPrepare * (envParams.puckRadius + envParams.malletRadius);
	xPrepare.x() = boost::algorithm::clamp(xPrepare.x(),
	                                       envParams.malletRadius + 0.02,
	                                       envParams.tableLength - envParams.malletRadius - 0.02);
	xPrepare.y() = boost::algorithm::clamp(xPrepare.y(),
	                                       -envParams.tableWidth / 2 + envParams.malletRadius + 0.02,
	                                       envParams.tableWidth / 2 - envParams.malletRadius - 0.02);
	double tStop = 0.08;
	bool success = false;

	for (int i = 0; i < 10; ++i) {
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();
		generator.combinatorialHit->plan(xStart2d, xPrepare, vPrepare,
		                                 state.cartTrajectory, tStop);
		generator.transformations->transformTrajectory(state.cartTrajectory);

		if (generator.optimizer->optimizeJointTrajectory(state.cartTrajectory, state.observation.jointPosition,
		                                                 state.jointTrajectory)) {
			state.cartTrajectory.header.stamp = tStart;
			state.jointTrajectory.header.stamp = tStart;
			return true;
		} else {
			vPrepare *= .8;
			ROS_INFO_STREAM("Optimization Failed [PREPARE]. Reduce the velocity: " << vPrepare.transpose());
		}
	}
	ROS_INFO_STREAM("Optimization Failed [PREPARE]. Failed to find a feasible hitting movement");
	return false;

}
