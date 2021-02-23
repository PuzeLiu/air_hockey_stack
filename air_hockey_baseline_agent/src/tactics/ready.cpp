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
#include <ros/ros.h>

using namespace Eigen;
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

Ready::Ready(EnvironmentParams &envParams, AgentParams &agentParams,
             SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Ready::ready() {
	return state.isNewTactics || !state.hasActiveTrajectory();
}

bool Ready::apply() {
	state.isNewTactics = false;

	Vector3d xStart, vStart;
	Vector2d xStart2d, vStart2d;
	ros::Time tStart;
	Kinematics::JointArrayType qStart, dqStart;

	state.getPlannedJointState(qStart, dqStart, tStart,
	                           agentParams.planTimeOffset);

	generator.getCartesianPosAndVel(xStart, vStart, qStart, dqStart);

	generator.transformations->applyInverseTransform(xStart);
	generator.transformations->applyInverseRotation(vStart);

	xStart2d = xStart.block<2, 1>(0, 0);
	vStart2d = vStart.block<2, 1>(0, 0);

	double tStop = std::max((agentParams.xHome.block<2, 1>(0, 0) - xStart2d).norm() / 0.5, 1 / 0.5);
	for (int i = 0; i < 10; ++i) {
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();
		generator.cubicLinearMotion->plan(xStart2d, vStart2d, agentParams.xHome.block<2, 1>(0, 0),
		                                  Vector2d(0., 0.), tStop, state.cartTrajectory);
		generator.transformations->transformTrajectory(state.cartTrajectory);
		if (!generator.optimizer->optimizeJointTrajectoryAnchor(
				state.cartTrajectory, qStart, agentParams.qHome, state.jointTrajectory)) {
			ROS_INFO_STREAM("Optimization Failed [READY]. Increase the motion time: " << tStop);
			tStop += 0.5;
		} else {
			state.jointTrajectory.header.stamp = tStart;
			state.cartTrajectory.header.stamp = tStart;
			return true;
		}
	}
	ROS_INFO_STREAM("Optimization Failed [READY]. Unable to find trajectory for Ready");
	return false;
}

Ready::~Ready() {

}

void Ready::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
		if (state.isPuckStatic()) {
			if (canSmash()) {
				setTactic(SMASH);
			} else if (puckStuck()) {
				setTactic(PREPARE);
			} else {
				setTactic(READY);
			}
		} else if (state.isPuckApproaching()) {
			if (shouldRepel()) {
				setTactic(REPEL);
			} else if (shouldCut()) {
				setTactic(CUT);
			} else {
				setTactic(READY);
			}
		} else {
			setTactic(READY);
		}
	}
}



