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
using namespace air_hockey_baseline_agent;

Cut::Cut(EnvironmentParams &envParams, AgentParams &agentParams,
         SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator) {
	collisionNumPrev = 0;
}

bool Cut::ready() {
	if (state.isNewTactics) {
		// Get planned position and velocity at now + timeoffset
		state.tPlan = ros::Time::now();
		state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset);
	} else {
		state.tStart = state.tPlan + ros::Duration(agentParams.defendPlanSteps / agentParams.rate);
	}
	return true;
}

bool Cut::apply() {
	double tStop;

	if (ros::Time::now() >= state.tPlan)
	{
        generator.getPlannedJointState(state, state.tStart);

        //! If the puck is too close, don't update the target point to avoid oscillation
        if (state.observation.puckEstimatedState.x() > agentParams.defendLine + envParams.malletRadius + 2 * envParams.puckRadius) {
            calculateCutPosition();
        }

        state.isNewTactics = false;

        state.trajectoryBuffer.getFree().cartTrajectory.points.clear();
        state.trajectoryBuffer.getFree().jointTrajectory.points.clear();

		tStop = std::max(agentParams.defendPlanSteps / agentParams.rate,
			(xCut - state.xPlan).norm() / agentParams.defendMaxEEVelocity);
		tStop = std::max(tStop, state.observation.puckPredictedState.predictedTime - 0.4);

		generator.cubicLinearMotion->plan(state.xPlan, state.vPlan, xCut, Vector3d(0., 0., 0.),
				tStop, agentParams.defendPlanSteps, state.trajectoryBuffer.getFree().cartTrajectory);

		generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);

		if (generator.optimizer->optimizeJointTrajectory(state.trajectoryBuffer.getFree().cartTrajectory,
														 state.qPlan, state.dqPlan,
														 state.trajectoryBuffer.getFree().jointTrajectory))
		{
            generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);

			state.tPlan = state.tStart;
            state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = state.tStart;
            state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = state.tStart;
			return true;
		}
		else
		{
			ROS_INFO_STREAM("Plan Failed");
			return false;
		}
	}
	return false;
}

Cut::~Cut() {

}

void Cut::setNextState() {
	if (not state.isPuckStatic() and (not state.isPuckApproaching() or not shouldCut())) {
		setTactic(READY);
		//! reset the cut record
		xCutPrev << 0., 0., 0.;
	}
}

void Cut::calculateCutPosition() {
	Vector2d cutTmp, vOut;
	Vector2d pos = state.observation.puckPredictedState.state.block<2, 1>(0, 0).transpose();
    Vector2d vIn = state.observation.puckPredictedState.state.block<2, 1>(2, 0);

	if (vIn[1] > 0.1 and pos[1] > 0.2) {
		ROS_INFO_STREAM("CUT2 LEFT");
		vOut << 0.1, 1.;
	} else if (vIn[1] < -0.1 and pos[1] < -0.2){
		ROS_INFO_STREAM("CUT2 RIGHT");
		vOut << 0.1, -1.;
	} else {
        vOut = - vIn;
		ROS_INFO_STREAM("BLOCK");
    }

    Vector2d offsetDir = (vIn - vOut).normalized();

    cutTmp = state.observation.puckPredictedState.state.block<2, 1>(0, 0) +
        offsetDir * (envParams.puckRadius + envParams.malletRadius);
    cutTmp.x() = boost::algorithm::clamp(cutTmp.x(), envParams.malletRadius + 0.02, envParams.tableLength / 2);
    cutTmp.y() = boost::algorithm::clamp(cutTmp.y(), -envParams.tableWidth / 2 + envParams.malletRadius + envParams.puckRadius,
        envParams.tableWidth / 2 - envParams.malletRadius - envParams.puckRadius);

	if (state.isNewTactics) {
		xCut.topRows(2) = cutTmp;
	} else {
		xCut.topRows(2) = (1 - agentParams.defendTargetUpdateRatio) * xCut.topRows(2) +
			agentParams.defendTargetUpdateRatio * cutTmp;
	}
	xCut[2] = agentParams.universalJointHeight;
	ROS_INFO_STREAM("Predict pos: " << pos.transpose() << " InDir: " << vIn.transpose() << " OurDir: " << vOut.transpose() << " offSetDir: " << offsetDir.transpose() << " cutTmp:" <<cutTmp << " cutTrue:" <<xCut);
}

