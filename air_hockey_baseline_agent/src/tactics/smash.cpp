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

Smash::Smash(EnvironmentParams &envParams, AgentParams &agentParams,
             SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator), gen(rd()), dist(0, 2) {
    hittingFailed = false;
}

bool Smash::ready() {
    if (state.isNewTactics) {
        hittingFailed = false;
        state.tPlan = ros::Time::now();
        state.tStart = state.tPlan + ros::Duration(agentParams.planTimeOffset);
    }
	return true;
}

bool Smash::apply() {
    if (state.isNewTactics) {
        state.isNewTactics = false;
        generator.getPlannedJointState(state, state.tStart);
        if (state.dqPlan.norm() > 0.2) {
            return generateStopTrajectory();
        }
    }
    if (ros::Time::now() >= state.tPlan) {
        generator.getPlannedJointState(state, state.tStart);
        return generateHitTrajectory(state.tStart);
    }
    return false;
}

Vector3d Smash::computeTarget(Vector3d puckPosition, int strategy) {
    Vector3d xTarget;
    if (puckPosition.y() > 0.1) {
        xTarget.y() = -0.05;
    } else if (puckPosition.y() < -0.1) {
        xTarget.y() = 0.05;
    } else {
        xTarget.y() = 0.0;
    }

    xTarget.x() = envParams.tableLength;
    xTarget.z() = 0.0;

    if (strategy == 1) {
        ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Strategy: Right");
        xTarget.y() = -2 * (envParams.tableWidth / 2 - envParams.puckRadius)
                      - agentParams.xGoal.y();
    } else if (strategy == 2) {
        ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Strategy: Left");
        xTarget.y() = 2 * (envParams.tableWidth / 2 - envParams.puckRadius)
                      - agentParams.xGoal.y();
    } else {
        ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Strategy: Middle");
    }

    return xTarget;
}

Vector3d Smash::computeTarget(Vector3d puckPosition) {
	if (isnan(agentParams.smashStrategy)){
        //use random strategy
        return computeTarget(puckPosition, dist(gen));
	} else{
        return computeTarget(puckPosition, agentParams.smashStrategy);
	}
}

void Smash::getHitPointVelocity(Vector2d &xHit2d, Vector2d &vHit2d,
                                JointArrayType &qHitRef) {
	Vector3d puckPredict;
	puckPredict.x() = state.observation.puckPredictedState.state.x();
	puckPredict.y() = state.observation.puckPredictedState.state.y();
	puckPredict.z() = 0.0;
	Vector3d xGoal = computeTarget(puckPredict);
	Vector3d vHit = (xGoal - puckPredict).normalized();
	Vector3d xHit = puckPredict - vHit * (envParams.puckRadius + envParams.malletRadius + 0.005);

	// optimize the hitting point
	double velMag;
	qHitRef = state.observation.jointPosition;
	generator.transformations->transformTable2Robot(xHit);
	generator.transformations->rotationTable2Robot(vHit);
	generator.hittingPointOptimizer->solve(xHit, vHit, qHitRef, velMag);
	velMag = std::max(std::min(velMag, agentParams.hitMaxVelocity), 0.);

	// prepare output
	generator.transformations->transformRobot2Table(xHit);
	generator.transformations->rotationRobot2Table(vHit);
	vHit = vHit * velMag * agentParams.hitVelocityScale;
	xHit2d = xHit.block<2, 1>(0, 0);
	vHit2d = vHit.block<2, 1>(0, 0);
}

Smash::~Smash() {

}

void Smash::setNextState() {
    if (ros::Time::now().toSec() > state.tNewTactics) {
		// Check if the optimized trajectory is in execution
		if (ros::Time::now() > state.trajectoryBuffer.getFree().jointTrajectory.header.stamp) {
			// Check if the hitting trajectory is finished
			if (ros::Time::now() > state.trajectoryBuffer.getExec().jointTrajectory.header.stamp +
								   state.trajectoryBuffer.getExec().jointTrajectory.points.back().time_from_start
				or hittingFailed) {
				setTactic(READY);
			}
		}
    }
}


bool Smash::generateHitTrajectory(ros::Time &tStart) {
	Vector2d vZero2d, xStop2d;
	vZero2d.setZero();
	double hitting_time;
    JointArrayType qHitRef(agentParams.nq);

    xCur2d = state.xPlan.block<2, 1>(0, 0);
    getHitPointVelocity(xHit2d, vHit2d, qHitRef);
    xStop2d = getStopPoint(xHit2d);

	for (size_t i = 0; i < 10; ++i) {
        state.trajectoryBuffer.getFree().cartTrajectory.points.clear();
        state.trajectoryBuffer.getFree().jointTrajectory.points.clear();

		if (not generator.combinatorialHitNew->plan(xCur2d, vZero2d, xHit2d, vHit2d, hitting_time,
													xStop2d, vZero2d, state.trajectoryBuffer.getFree().cartTrajectory))	{
			break;
		}

		generator.transformations->transformTrajectory(state.trajectoryBuffer.getFree().cartTrajectory);
		if (generator.optimizer->optimizeJointTrajectoryAnchor(state.trajectoryBuffer.getFree().cartTrajectory,
			state.qPlan, state.dqPlan, qHitRef, hitting_time, state.trajectoryBuffer.getFree().jointTrajectory)) {
//		if (generator.optimizer->optimizeJointTrajectory(state.trajectoryBuffer.getFree().cartTrajectory,
//															   state.qPlan, state.dqPlan,
//															   state.trajectoryBuffer.getFree().jointTrajectory)) {

            generator.cubicSplineInterpolation(state.trajectoryBuffer.getFree().jointTrajectory, state.planPrevPoint);

			if (ros::Time::now() > tStart) {
				tStart = ros::Time::now();
			}

			// Check if the hitting trajectory is starting too early
			auto tHitStart = state.observation.puckPredictedState.stamp +
                ros::Duration(state.observation.puckPredictedState.predictedTime) -
                state.trajectoryBuffer.getFree().jointTrajectory.points.back().time_from_start;
			if (tStart < tHitStart) {
				tStart = tHitStart;
			}

            state.tPlan = tStart + state.trajectoryBuffer.getFree().jointTrajectory.points.back().time_from_start;
            state.trajectoryBuffer.getFree().jointTrajectory.header.stamp = tStart;
            state.trajectoryBuffer.getFree().cartTrajectory.header.stamp = tStart;
            hittingFailed = false;
			return true;
		}
		else {
			vHit2d *= 0.9;
		}
	}

	vHit2d = vHit2d * 0.;
    state.tPlan = ros::Time::now() + ros::Duration(agentParams.planTimeOffset);
	state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
	state.trajectoryBuffer.getFree() = state.trajectoryBuffer.getExec();
	ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " +	"Failed to find a feasible movement [HITTING]");
    hittingFailed = true;
	return false;
}

Vector2d Smash::getStopPoint(Eigen::Vector2d hitPoint) {
	Vector2d stopPoint;
	if (hitPoint.y() > 0) {
		stopPoint = hitPoint + Vector2d(0.2, -0.15);
	} else {
		stopPoint = hitPoint + Vector2d(0.2, 0.15);
	}

	stopPoint.x() = fmin(stopPoint.x(), agentParams.hitRange[1]);
	stopPoint.y() = fmax(fmin(stopPoint.y(), envParams.tableWidth / 2 - envParams.malletRadius - 0.05),
	                     -envParams.tableWidth / 2 + envParams.malletRadius + 0.05);
	return stopPoint;
}