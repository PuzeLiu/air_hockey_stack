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

Cut::Cut(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool Cut::ready() {
	return state.restart;
}

bool Cut::apply() {
	Vector2d xCut(agentParams.defendLine_,
			state.observation.puckPredictedState.state.y());

	Vector3d xCur, vCur;
	Vector2d xCur2d, vCur2d;
	ros::Time tStart;
	Kinematics::JointArrayType qStart, dqStart;

	state.getPlannedState(xCur, vCur, qStart, dqStart, tStart,
			agentParams.planTimeOffset_);

	generator.transformations->applyInverseTransform(xCur);
	generator.transformations->applyInverseRotation(vCur);

	xCur2d = xCur.block<2, 1>(0, 0);
	vCur2d = vCur.block<2, 1>(0, 0);
	double tStop = boost::algorithm::clamp(
			observationState_.puckPredictedState.predictedTime - 0.3,
			agentParams.tDefendMin_, agentParams.maxPredictionTime_);
	xCut.y() = boost::algorithm::clamp(xCut.y(),
			-envParams.tableWidth_ / 2 + envParams.malletRadius_ + 0.02,
			envParams.tableWidth_ / 2 - envParams.malletRadius_ - 0.02);

	for (int i = 0; i < 10; ++i) {
		state.cartTrajectory_.points.clear();
		state.jointTrajectory_.points.clear();
		generator.cubicLinearMotion->plan(xCur2d, vCur2d, xCut,
				Vector2d(0., 0.), tStop, state.cartTrajectory_);
		generator.transformations->transformTrajectory(state.cartTrajectory_);

		bool ok = generator.optimizer->optimizeJointTrajectory(
				state.cartTrajectory_, qStart,
				//            qAnchor << 0., 0., 0., 0., 0., 0, 0.;
				state.jointTrajectory_);

		if (!ok) {
			ROS_INFO_STREAM(
					"Optimization Failed [Cut]. Increase the motion time: " << tStop);
			tStop += 0.1;
		} else {
			state.jointTrajectory_.header.stamp = tStart;
			state.cartTrajectory_.header.stamp = tStart;
			state.trajStopTime_ = state.jointTrajectory_.header.stamp
					+ ros::Duration(tStop);
			return true;
		}
	}

	ROS_INFO_STREAM("Optimization Failed [Cut].");
}

Cut::~Cut() {

}

