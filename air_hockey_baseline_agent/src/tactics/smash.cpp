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

using namespace air_hockey_baseline_agent;
using namespace Eigen;

Smash::Smash(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState &state, TrajectoryGenerator *generator) :
		Tactic(envParams, agentParams, state, generator), gen(rd()), dist(0, 2) {

}

bool Smash::ready() {
    return state.isNewTactics;
}

bool Smash::apply() {
	state.isNewTactics = false;

	getHitPointVelocity(xCur2d, xHit2d, vHit2d, qHitRef);

	bool success = false;

	for (size_t i = 0; i < 10; ++i) {
		state.cartTrajectory.points.clear();
		state.jointTrajectory.points.clear();

		bool ok = generator.combinatorialHit->plan(xCur2d, xHit2d, vHit2d,
				state.cartTrajectory);
		if (!ok) {
			return false;
		}
		generator.transformations->transformTrajectory(state.cartTrajectory);
		if (generator.optimizer->optimizeJointTrajectoryAnchor(state.cartTrajectory, state.observation.jointPosition,
														 qHitRef, state.jointTrajectory)){
//		if (generator.optimizer->optimizeJointTrajectory(state.cartTrajectory,
//				state.observation.jointPosition, state.jointTrajectory)) {
			ROS_INFO_STREAM("[HITTING] velocity: " << vHit2d.norm());
			success = true;
			break;
		} else {
            vHit2d *= .8;
			ROS_INFO_STREAM(
					"Optimization Failed [HITTING]. Reduce the velocity: " << vHit2d.transpose());
			continue;
		}
	}
	//! plan for return trajectory
	trajectory_msgs::MultiDOFJointTrajectory cartTrajReturn;
	trajectory_msgs::JointTrajectory jointTrajReturn;
	double vMax = 0.5;
	success = success && planReturnTraj(vMax, cartTrajReturn, jointTrajReturn);

	//! append return to whole trajectory
	if (success) {
		state.cartTrajectory.points.insert(state.cartTrajectory.points.end(),
				cartTrajReturn.points.begin(), cartTrajReturn.points.end());
		state.jointTrajectory.points.insert(state.jointTrajectory.points.end(),
				jointTrajReturn.points.begin(), jointTrajReturn.points.end());
		state.jointTrajectory.header.stamp = ros::Time::now();
		return true;
	} else {
		ROS_INFO_STREAM("Failed to find a feasible movement [HITTING]");
		return false;
	}
}

Vector3d Smash::computeTarget(Vector3d puckPosition) {
	Vector3d xTarget;
	auto random_integer = dist(gen);
	random_integer = 0;
	if (puckPosition.y() > 0.1) {
		xTarget.y() = -0.1;
	} else if (puckPosition.y() < -0.1) {
		xTarget.y() = 0.1;
	} else {
		xTarget.y() = 0.0;
	}

    xTarget.x() = envParams.tableLength;
	xTarget.z() = 0.0;

	if (random_integer == 1) {
		ROS_INFO_STREAM("Strategy: Right");
		xTarget.y() = -2 * (envParams.tableWidth / 2 - envParams.puckRadius)
				- agentParams.xGoal.y();
	} else if (random_integer == 2) {
		ROS_INFO_STREAM("Strategy: Left");
		xTarget.y() = 2 * (envParams.tableWidth / 2 - envParams.puckRadius)
				- agentParams.xGoal.y();
	} else {
		ROS_INFO_STREAM("Strategy: Middle");
	}

	return xTarget;
}

void Smash::getHitPointVelocity(Vector2d &xCur2d, Vector2d &xHit2d, Vector2d &vHit2d,
                                iiwas_kinematics::Kinematics::JointArrayType &qHitRef){
    Vector3d xCur;
    generator.kinematics->forwardKinematics(state.observation.jointPosition,
                                            xCur);
    generator.transformations->applyInverseTransform(xCur);
    Vector3d puckCur;
    puckCur.x() = state.observation.puckPredictedState.state.x();
    puckCur.y() = state.observation.puckPredictedState.state.y();
    puckCur.z() = 0.0;
    Vector3d xGoal = computeTarget(puckCur);
    Vector3d vHit = (xGoal - puckCur).normalized();
    Vector3d xHit = puckCur - vHit * (envParams.puckRadius + envParams.malletRadius + 0.005);

    // optimize the hitting point
    double velMag;
    qHitRef = state.observation.jointPosition;
    generator.transformations->applyForwardTransform(xHit);
    generator.transformations->applyForwardRotation(vHit);
    generator.hittingPointOptimizer->solve(xHit, vHit, qHitRef, velMag);


    // prepare output
    generator.transformations->applyInverseTransform(xHit);
    generator.transformations->applyInverseRotation(vHit);
    vHit = vHit * velMag;
    xCur2d = xCur.block<2, 1>(0, 0);
    xHit2d = xHit.block<2, 1>(0, 0);
    vHit2d = vHit.block<2, 1>(0, 0);
//    vHit2d *= agentParams.vHitMax;
}

Smash::~Smash() {

}

void Smash::setNextState() {
	if (state.hasActiveTrajectory()){
		setTactic(SMASH);
	} else {
		setTactic(READY);
	}
}
