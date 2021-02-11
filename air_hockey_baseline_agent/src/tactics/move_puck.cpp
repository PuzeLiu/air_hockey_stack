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

using namespace air_hockey_baseline_agent;

MovePuck::MovePuck(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState *state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool MovePuck::ready(SystemState &state) {
	return ros::Time::now() > state.trajStopTime_;
}

bool MovePuck::apply(SystemState &state) {
	Vector3d xCur;
	kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
	applyInverseTransform(xCur);

	Vector3d xLiftUp, xSetDown;
	xLiftUp = observationState_.puckPredictedState.state.block<3, 1>(0, 0);
	xLiftUp.x() = boost::algorithm::clamp(xLiftUp.x(), malletRadius_ + 0.02,
			tableLength_ - malletRadius_ - 0.02);
	xLiftUp.y() = boost::algorithm::clamp(xLiftUp.y(),
			-tableWidth_ / 2 + malletRadius_ + 0.02,
			tableWidth_ / 2 - malletRadius_ - 0.02);
	xLiftUp.z() = prepareHeight_;
	xSetDown = xLiftUp;
	xSetDown.z() = universalJointHeight_ + puckHeight_;

	double tStop = 2.0;
	for (int i = 0; i < 10; ++i) {
		cartTrajectory_.points.clear();
		jointTrajectory_.points.clear();

		cubicLinearMotion_->plan(xCur, Vector3d(0., 0., 0.), xLiftUp,
				Vector3d(0., 0., 0.), tStop, cartTrajectory_);
		cubicLinearMotion_->plan(xLiftUp, Vector3d(0., 0., 0.), xSetDown,
				Vector3d(0., 0., 0.), tStop, cartTrajectory_);
		cubicLinearMotion_->plan(xSetDown, Vector3d(0., 0., 0.), xPrepare_,
				Vector3d(0., 0., 0.), tStop, cartTrajectory_);
		generator.transformations->transformTrajectory(cartTrajectory_);
		if (!optimizer_->optimizeJointTrajectory(cartTrajectory_,
				observationState_.jointPosition, jointTrajectory_)) {
			ROS_INFO_STREAM(
					"Optimization Failed [PREPARE]. Increase the motion time: " << tStop);
			tStop += 0.2;
		} else {
			jointTrajectory_.header.stamp = ros::Time::now();
			trajStopTime_ = jointTrajectory_.header.stamp
					+ ros::Duration(3 * tStop);
			ros::Duration(3 * tStop).sleep();
			return;
		}
	}
	ROS_INFO_STREAM(
			"Optimization Failed [PREPARE]. Unable to find trajectory for Prepare");

}

MovePuck::~MovePuck() {

}

