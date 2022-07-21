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

using namespace Eigen;
using namespace trajectory_msgs;
using namespace air_hockey_baseline_agent;

Prepare::Prepare(EnvironmentParams &envParams, AgentParams &agentParams,
                 SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Prepare::ready() {
	if (state.isNewTactics) {
		state.tPlan = ros::Time::now();
		state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset);
	}
	return true;
}

bool Prepare::apply() {
	if (state.isNewTactics) {
		state.isNewTactics = false;
		generator.getPlannedJointState(state, state.tStart);
		if (state.dqPlan.norm() > 0.2) {
			return generateStopTrajectory();
		}
	}

	if (ros::Time::now() >= state.tPlan) {
		generator.getPlannedJointState(state, state.tStart);
		return generatePrepareTrajectory(state.qPlan, state.dqPlan);
	}
	return false;
}

Prepare::~Prepare() {

}

void Prepare::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
		// Check if the optimized trajectory is in execution
		if (ros::Time::now() > state.trajectoryBuffer.getFree().jointTrajectory.header.stamp) {
			// Check if the planned trajectory is finished
			if (ros::Time::now() > state.trajectoryBuffer.getExec().jointTrajectory.header.stamp +
								   state.trajectoryBuffer.getExec().jointTrajectory.points.back().time_from_start){
				setTactic(READY);
			}
		}
	}
}

bool Prepare::generatePrepareTrajectory(JointArrayType &qStart, JointArrayType &dqStart) {
	double hitting_time;
	Vector2d xStart2d, xPuck, xPrepare, vPrepare, xStop2d, vZero2d;
	vZero2d.setZero();

	getPreparePosAndVel(xStart2d, xPuck, xPrepare, vPrepare, xStop2d);
	double tPreMotion = (xStart2d - state.xPlan.block<2, 1>(0, 0)).norm() / agentParams.prepareVelocity;
	tPreMotion = std::max(tPreMotion, 0.2);

	state.trajectoryBuffer.getFree().cartTrajectory.points.clear();
	state.trajectoryBuffer.getFree().jointTrajectory.points.clear();

	generator.cubicLinearMotion->plan(state.xPlan.block<2, 1>(0, 0), vZero2d,
									  xStart2d, vZero2d,
									  tPreMotion, state.trajectoryBuffer.getFree().cartTrajectory);
	generator.combinatorialHitNew->plan(xStart2d, vZero2d,
										xPrepare, vPrepare, hitting_time,
										xStop2d, vZero2d,
										state.trajectoryBuffer.getFree().cartTrajectory);
	generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);

	if (generator.optimizer->optimizeJointTrajectory(state.trajectoryBuffer.getFree().cartTrajectory,
													 state.qPlan, state.dqPlan,
													 state.trajectoryBuffer.getFree().jointTrajectory)) {
		generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);
		generator.synchronizeCartesianTrajectory(state.trajectoryBuffer.getFree().jointTrajectory,
												 state.trajectoryBuffer.getFree().cartTrajectory);

		if (ros::Time::now() > state.tStart) {
			state.tStart = ros::Time::now();
		}

		// Check if the hitting trajectory is starting too early
		auto tHitStart = state.observation.puckPredictedState.stamp +
						 ros::Duration(state.observation.puckPredictedState.predictedTime) -
						 state.trajectoryBuffer.getFree().jointTrajectory.points.back().time_from_start;
		if (state.tStart < tHitStart) {
			state.tStart = tHitStart;
		}

		state.tPlan = state.tStart + state.trajectoryBuffer.getFree().jointTrajectory.points.back().time_from_start + ros::Duration(agentParams.planTimeOffset);
		state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = state.tStart;
		state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = state.tStart;
		return true;
	} else {
		vPrepare = vPrepare * 0.;
		state.tPlan = ros::Time::now() + ros::Duration(agentParams.planTimeOffset);
		state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
		state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
		ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
												"Optimization Failed [PREPARE]. Failed to find a feasible hitting movement");
		setTactic(READY);
		return false;
	}

}

void Prepare::getPreparePosAndVel(Vector2d &xStart, Vector2d &xPuck, Vector2d &xPrepare, Vector2d &vPrepare, Vector2d &xStop) {
	xPuck = state.observation.puckPredictedState.state.block<2, 1>(0, 0);
	if (xPuck.x() < agentParams.xHome.x() + 0.02){
		vPrepare.x() = -1.0;
		vPrepare.y() = copysign(0.5, xPuck.y());
	}
	else if (abs(xPuck.y()) > envParams.tableWidth / 2 - envParams.puckRadius - 0.02) {
		vPrepare.y() = copysign(0.2, xPuck.y());
		vPrepare.x() = 1.0;
	}
	else if (xPuck.x() > 0.6) {
		vPrepare.x() = -0.1;
		vPrepare.y() = copysign(1., xPuck.y());
	}
	else if (xPuck.x() < 0.2){
		vPrepare.x() = 0.1;
		vPrepare.y() = copysign(1., xPuck.y());
	} else {
		vPrepare.x() = 0.;
		vPrepare.y() = copysign(1., xPuck.y());
	}
	vPrepare.normalize();

	xStart = xPuck - vPrepare * 0.2;
	xPrepare = xPuck - vPrepare * (envParams.puckRadius + envParams.malletRadius);
	xStop = xPrepare + (agentParams.xHome.block<2, 1>(0, 0) - xPrepare).normalized() * 0.2;
	xPrepare.x() = boost::algorithm::clamp(xPrepare.x(),
	                                       envParams.malletRadius + envParams.puckRadius + 0.02,
	                                       envParams.tableLength / 2 - envParams.malletRadius -
										   envParams.puckRadius - 0.02);
	xPrepare.y() = boost::algorithm::clamp(xPrepare.y(),
	                                       -envParams.tableWidth / 2 + envParams.malletRadius +
										   envParams.puckRadius + 0.02,
	                                       envParams.tableWidth / 2 - envParams.malletRadius -
										   envParams.puckRadius - 0.02);

	xStop.x() = boost::algorithm::clamp(xStop.x(),
										   envParams.malletRadius + envParams.puckRadius + 0.02,
										   envParams.tableLength / 2 - envParams.malletRadius -
										   envParams.puckRadius - 0.02);
	xStop.y() = boost::algorithm::clamp(xStop.y(),
										   -envParams.tableWidth / 2 + envParams.malletRadius +
										   envParams.puckRadius + 0.02,
										   envParams.tableWidth / 2 - envParams.malletRadius -
										   envParams.puckRadius - 0.02);
	vPrepare = vPrepare * agentParams.prepareVelocity;

}
