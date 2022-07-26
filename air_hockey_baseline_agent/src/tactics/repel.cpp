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

using namespace Eigen;
using namespace air_hockey_baseline_agent;

Repel::Repel(EnvironmentParams &envParams, AgentParams &agentParams,
			 SystemState &state, TrajectoryGenerator *generator) :
	Tactic(envParams, agentParams, state, generator) {
	hittingFailed = false;
}

bool Repel::ready() {
	if (state.isNewTactics) {
		// Get planned position and velocity at now + timeoffset
		state.tPlan = ros::Time::now();
		state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset);
		hittingFailed = false;
	}
	return true;
}

bool Repel::apply() {
	if (state.isNewTactics) {
		state.isNewTactics = false;
		generator.getPlannedJointState(state, state.tStart);
		if (state.vPlan.norm() > 0.01) {
			return generateStopTrajectory();
		}
	}

	if (ros::Time::now() >= state.tPlan) {
		generator.getPlannedJointState(state, state.tStart);
		return generateRepelTrajectory();
	}
	return false;
}

Repel::~Repel() {

}

void Repel::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
		if (hittingFailed) setTactic(READY);
		// Check if the optimized trajectory is in execution
		if (ros::Time::now() > state.trajectoryBuffer.getFree().jointTrajectory.header.stamp) {
			// Check if the hitting trajectory is finished
			if (ros::Time::now() > state.trajectoryBuffer.getExec().jointTrajectory.header.stamp +
				state.trajectoryBuffer.getExec().jointTrajectory.points.back().time_from_start) {
				setTactic(READY);
			}
		}
	}
}

bool Repel::generateRepelTrajectory() {
	Vector2d vZero2d, xHit2d, vHit2d, xStop2d;
	vZero2d.setZero();
	Vector3d xCur;
	double hitting_time;
	double repel_velocity = 1.0;

	for (size_t i = 0; i < 10; ++i) {
		state.trajectoryBuffer.getFree().cartTrajectory.points.clear();
		state.trajectoryBuffer.getFree().jointTrajectory.points.clear();

		state.observer->updateObservation(0.5);
		xHit2d = state.observation.puckPredictedState.state.block<2, 1>(0, 0);
		vHit2d = -state.observation.puckPredictedState.state.block<2, 1>(2, 0).normalized();
		xStop2d = getStopPoint(xHit2d, vHit2d);
		vHit2d = vHit2d * repel_velocity;

		if (!generator.combinatorialHitNew->plan(state.xPlan.block<2, 1>(0, 0),
												 state.vPlan.block<2, 1>(0, 0), xHit2d, vHit2d, hitting_time,
												 xStop2d, vZero2d, state.trajectoryBuffer.getFree().cartTrajectory)) {
			ROS_INFO_STREAM("Plan Failed");
			ROS_INFO_STREAM(state.xPlan.transpose()
								<< " v: " << state.vPlan.transpose() << " xHit: " << xHit2d.transpose()
								<< " vHit: " << vHit2d.transpose() << " xStop: " << xStop2d.transpose());
			break;
		}
		generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);

		if (generator.optimizer->optimizeJointTrajectoryAnchor(state.trajectoryBuffer.getFree().cartTrajectory,
														 state.qPlan, state.dqPlan, state.qPlan, hitting_time,
														 state.trajectoryBuffer.getFree().jointTrajectory)) {
			auto tPuckHit = state.observation.stamp +
				ros::Duration(state.observation.puckPredictedState.predictedTime);

			if (state.tStart < ros::Time::now()) state.tStart = ros::Time::now();

			if (state.tStart + ros::Duration(hitting_time) < tPuckHit) {
				double timeScaleFactor = (tPuckHit - ros::Time::now()).toSec() / hitting_time;
				timeScaleFactor = std::max(timeScaleFactor, 2.0);

				for (int j = 0; j < state.trajectoryBuffer.getFree().jointTrajectory.points.size(); ++j) {
					state.trajectoryBuffer.getFree().cartTrajectory.points[j].time_from_start *= timeScaleFactor;
					state.trajectoryBuffer.getFree().jointTrajectory.points[j].time_from_start *= timeScaleFactor;
				}
				hitting_time = timeScaleFactor * hitting_time;
			}

			generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);
			generator.synchronizeCartesianTrajectory(state.trajectoryBuffer.getFree().jointTrajectory,
													 state.trajectoryBuffer.getFree().cartTrajectory);

			state.tPlan = state.tStart + state.trajectoryBuffer.getFree().jointTrajectory.points.back().time_from_start;
			state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = state.tStart;
			state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = state.tStart;
			hittingFailed = false;
			return true;
		} else {
			repel_velocity *= .8;
		}
	}

	vHit2d = vHit2d * 0.;
	state.tPlan = ros::Time::now() + ros::Duration(agentParams.planTimeOffset);
	state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
	state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
	ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Optimization Failed [REPEL]");
	hittingFailed = true;
	return false;
}

Vector2d Repel::getStopPoint(Eigen::Vector2d &hitPoint, Eigen::Vector2d &hitDirection) {
	Vector2d stopPoint;
	stopPoint = hitPoint + 0.05 * hitDirection;

	stopPoint.x() = fmin(stopPoint.x(), agentParams.hitRange[1]);
	stopPoint.y() = fmax(fmin(stopPoint.y(), envParams.tableWidth / 2 - envParams.malletRadius - 0.05),
						 -envParams.tableWidth / 2 + envParams.malletRadius + 0.05);
	return stopPoint;
}