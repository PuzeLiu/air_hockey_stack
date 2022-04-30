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

}

bool Smash::ready() {
	return state.isNewTactics;
}

bool Smash::apply() {
	JointArrayType qStart(agentParams.nq), dqStart(agentParams.nq);
	ros::Time tStart;
	state.getPlannedJointState(qStart, dqStart, tStart, agentParams.planTimeOffset);

	if (dqStart.norm() > 0.05) {
		generateStopTrajectory();
		return true;
	} else {
		state.isNewTactics = false;
		bool success = generateHitTrajectory(qStart, tStart);
		return success;
	}
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
		if (state.hasActiveTrajectory()) {
			setTactic(SMASH);
		} else {
			setTactic(READY);
		}
	}
}


bool Smash::generateHitTrajectory(const JointArrayType &qCur, ros::Time &tStart) {
	JointArrayType qHitRef(agentParams.nq);

	Vector3d xCur;
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, qCur);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	xCur = generator.agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();

	generator.transformations->transformRobot2Table(xCur);
	xCur2d = xCur.block<2, 1>(0, 0);

	getHitPointVelocity(xHit2d, vHit2d, qHitRef);

	Vector2d vZero2d, xStop2d;
	vZero2d.setZero();
	xStop2d = getStopPoint(xHit2d);
	double hitting_time;

//	trajectory_msgs::MultiDOFJointTrajectory cartTrajStop;
//	trajectory_msgs::JointTrajectory jointTrajStop;
//	cartTrajStop.joint_names = state.cartTrajectory.joint_names;
//	jointTrajStop.joint_names = state.jointTrajectory.joint_names;

	for (size_t i = 0; i < 10; ++i) {
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();
//		cartTrajStop.points.clear();
//		jointTrajStop.points.clear();

//		//! ############ Phase 1 ############
//		Vector2d vZero;
//		vZero.setZero();
//		if (!generator.combinatorialHitNew->plan(xCur2d, vZero, xHit2d, vHit2d, state.cartTrajectory)) {
//			return false;
//		}
//
//		Vector2d xPhase2Start(state.cartTrajectory.points.back().transforms[0].translation.x,
//		                      state.cartTrajectory.points.back().transforms[0].translation.y);
//		Vector2d vPhase2Start(state.cartTrajectory.points.back().velocities[0].linear.x,
//		                      state.cartTrajectory.points.back().velocities[0].linear.y);
//		Vector2d xStop2d = getStopPoint(xHit2d);
//		cartTrajStop.points.push_back(state.cartTrajectory.points.back());
//
//		generator.transformations->transformTrajectory(state.cartTrajectory);
//
////		bool success = generator.optimizer->optimizeJointTrajectory(state.cartTrajectory, qCur, state.jointTrajectory);
//		bool success = generator.optimizer->optimizeJointTrajectoryAnchor(state.cartTrajectory, qCur,
//		                                                                  qHitRef, state.jointTrajectory,
//		                                                                  true);
//		if (!success) {
//			vHit2d *= .9;
//			ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
//					"Optimization Failed [HITTING Phase 1]. Reduce the velocity: " << vHit2d.transpose());
//			continue;
//		}
//
//		//! ############ Phase 2 ############
//		if (!generator.combinatorialHitNew->plan(xPhase2Start, vPhase2Start, xStop2d, vZero, cartTrajStop)) {
//			return false;
//		}
//		generator.transformations->transformTrajectory(cartTrajStop);
//		cartTrajStop.points.erase(cartTrajStop.points.begin());
//
//		//! Trajectory Optimization for second part
//		JointArrayType qHitOpt = JointArrayType::Map(state.jointTrajectory.points.back().positions.data(),
//		                                             state.jointTrajectory.points.back().positions.size());
//		success = success && generator.optimizer->optimizeJointTrajectoryAnchor(cartTrajStop, qHitOpt, qHitRef,
//		                                                                        jointTrajStop, false);
//		if (!success) {
//			vHit2d *= .9;
//			ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
//					"Optimization Failed [HITTING Phase 2]. Reduce the velocity: " << vHit2d.transpose());
//			continue;
//		} else {
//			//! Concatenate trajectory
//			state.cartTrajectory.points.insert(state.cartTrajectory.points.end(),
//			                                   cartTrajStop.points.begin(),
//			                                   cartTrajStop.points.end());
//			state.jointTrajectory.points.insert(state.jointTrajectory.points.end(),
//			                                    jointTrajStop.points.begin(),
//			                                    jointTrajStop.points.end());
////			generator.interpolateAcceleration(state.jointTrajectory);
////			generator.interpolateVelocity(state.jointTrajectory);
////			generator.cubicSplineInterpolation(state.jointTrajectory);
//
//			if (ros::Time::now() > tStart) {
//				tStart = ros::Time::now();
//			}
//
//			auto tHitStart = state.observation.stamp +
//			                 ros::Duration(agentParams.tPredictionMax) -
//			                 state.jointTrajectory.points.back().time_from_start;
//
//			if (tStart < tHitStart) {
//				tStart = tHitStart;
//			}
//
//			state.jointTrajectory.header.stamp = tStart;
//			state.cartTrajectory.header.stamp = tStart;
//			return true;
//		}

		if (not generator.combinatorialHitNew->plan(xCur2d, vZero2d, xHit2d, vHit2d,
			xStop2d, vZero2d, hitting_time, state.cartTrajectory))	{
			return false;
		}

		generator.transformations->transformTrajectory(state.cartTrajectory);
		if (generator.optimizer->optimizeJointTrajectoryAnchor(state.cartTrajectory,
			qCur, qHitRef, hitting_time, state.jointTrajectory)) {
			if (ros::Time::now() > tStart) {
				tStart = ros::Time::now();

			}
			auto tHitStart = state.observation.stamp + ros::Duration(agentParams.tPredictionMax) -
				state.jointTrajectory.points.back().time_from_start;

			if (tStart < tHitStart) {
				tStart = tHitStart;
			}

			state.jointTrajectory.header.stamp = tStart;
			state.cartTrajectory.header.stamp = tStart;
			return true;
		}
		else {
			vHit2d *= 0.9;
		}
	}

	vHit2d = vHit2d * 0.;
	state.jointTrajectory.points.clear();
	state.cartTrajectory.points.clear();
	ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
			"Failed to find a feasible movement [HITTING]");
	return false;
}

Vector2d Smash::getStopPoint(Eigen::Vector2d hitPoint) {
	Vector2d stopPoint;
	if (hitPoint.y() > 0) {
		stopPoint = hitPoint + Vector2d(0.2, -0.25);
	} else {
		stopPoint = hitPoint + Vector2d(0.2, 0.25);
	}

	stopPoint.x() = fmin(stopPoint.x(), agentParams.hitRange[1]);
	stopPoint.y() = fmax(fmin(stopPoint.y(), envParams.tableWidth / 2 - envParams.malletRadius - 0.05),
	                     -envParams.tableWidth / 2 + envParams.malletRadius + 0.05);
	return stopPoint;
}