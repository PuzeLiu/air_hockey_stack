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

MovePuck::MovePuck(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {

}

bool MovePuck::ready() {
	return state.isNewTactics;
}

bool MovePuck::apply() {
	state.isNewTactics = false;

	Vector3d xCur;
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, state.observation.jointPosition);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	xCur = generator.agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();
	generator.transformations->transformRobot2Table(xCur);

	Vector3d xLiftUp, xSetDown;
	xLiftUp = state.observation.puckPredictedState.state.block<3, 1>(0, 0);

	xLiftUp.x() = boost::algorithm::clamp(xLiftUp.x(),
			envParams.malletRadius + 0.02,
			envParams.tableLength - envParams.malletRadius - 0.02);
	xLiftUp.y() = boost::algorithm::clamp(xLiftUp.y(),
			-envParams.tableWidth / 2 + envParams.malletRadius + 0.02,
			envParams.tableWidth / 2 - envParams.malletRadius - 0.02);
	xLiftUp.z() = agentParams.initHeight;
	xSetDown = xLiftUp;
	xSetDown.z() = agentParams.universalJointHeight + envParams.puckHeight;

	double tStop = 2.0;
	for (int i = 0; i < 10; ++i) {
        state.trajectoryBuffer.getFree().cartTrajectory.points.clear();
        state.trajectoryBuffer.getFree().jointTrajectory.points.clear();

		generator.cubicLinearMotion->plan(xCur, Vector3d(0., 0., 0.), xLiftUp,
				Vector3d(0., 0., 0.), tStop, state.trajectoryBuffer.getFree().cartTrajectory);
		generator.cubicLinearMotion->plan(xLiftUp, Vector3d(0., 0., 0.),
				xSetDown, Vector3d(0., 0., 0.), tStop, state.trajectoryBuffer.getFree().cartTrajectory);
		generator.cubicLinearMotion->plan(xSetDown, Vector3d(0., 0., 0.),
				agentParams.xInit, Vector3d(0., 0., 0.), tStop,
            state.trajectoryBuffer.getFree().cartTrajectory);

		generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);

		bool ok = generator.optimizer->optimizeJointTrajectory(
            state.trajectoryBuffer.getFree().cartTrajectory, state.observation.jointPosition,
            state.trajectoryBuffer.getFree().jointTrajectory);
		if (!ok) {
			ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
					"Optimization Failed [PREPARE]. Increase the motion time: " << tStop);
			tStop += 0.2;
		} else {
            state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = ros::Time::now();
			return true;
		}
	}
	ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
			"Optimization Failed [PREPARE]. Unable to find trajectory for Prepare");

	return false;

}

MovePuck::~MovePuck() {

}

void MovePuck::setNextState() {
	if (ros::Time::now().toSec() > state.tNewTactics) {
		if (state.hasActiveTrajectory()) {
			setTactic(READY);
		} else {
			setTactic(READY);
		}
	}
}

