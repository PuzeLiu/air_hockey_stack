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

#include <ros/ros.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace air_hockey_baseline_agent;

Tactic::Tactic(EnvironmentParams &envParams, AgentParams &agentParams,
               SystemState &state, TrajectoryGenerator *generator) :
		envParams(envParams), agentParams(agentParams), state(state), generator(
		*generator) {

}

bool Tactic::planReturnTraj(const double &vMax,
                            trajectory_msgs::MultiDOFJointTrajectory &cartTrajReturn,
                            trajectory_msgs::JointTrajectory &jointTrajReturn) {
	double vReadyMax = vMax;
	cartTrajReturn.joint_names.push_back("x");
	cartTrajReturn.joint_names.push_back("y");
	cartTrajReturn.joint_names.push_back("z");

	trajectory_msgs::MultiDOFJointTrajectoryPoint lastPoint =
			state.cartTrajectory.points.back();

	for (size_t i = 0; i < 10; ++i) {
		cartTrajReturn.points.clear();
		jointTrajReturn.points.clear();
		cartTrajReturn.points.push_back(lastPoint);

		Vector3d xStop;
		xStop
				<< lastPoint.transforms[0].translation.x, lastPoint.transforms[0].translation.y, lastPoint.transforms[0].translation.z;
		generator.transformations->transformRobot2Table(xStop);
		Vector2d xStop2d = xStop.block<2, 1>(0, 0);
		Vector2d xHome2d = agentParams.xHome.block<2, 1>(0, 0);
		double tStop = (xHome2d - xStop2d).norm() / vReadyMax;
		generator.cubicLinearMotion->plan(xStop2d, Vector2d(0., 0.),
		                                  xHome2d, Vector2d(0., 0.), tStop, cartTrajReturn);
		cartTrajReturn.points.erase(cartTrajReturn.points.begin());
		generator.transformations->transformTrajectory(cartTrajReturn);

		JointArrayType qStart(agentParams.nq);
		for (int j = 0; j < generator.agentParams.nq; ++j) {
			qStart[j] = state.jointTrajectory.points.back().positions[j];
		}

		bool ok = generator.optimizer->optimizeJointTrajectoryAnchor(
				cartTrajReturn, qStart, agentParams.qHome, jointTrajReturn);

		if (ok) {
			return true;
		} else {
			vReadyMax *= 0.8;
			ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " +
					"Optimization Failed [RETURN]. Reduce the velocity for Ready: " << vReadyMax);
		}
	}

	return false;
}

void Tactic::generateStopTrajectory() {
	JointArrayType q(agentParams.nq), dq(agentParams.nq);
	ros::Time tTmp;
	state.getPlannedJointState(q, dq, tTmp, agentParams.planTimeOffset / 2);

	trajectory_msgs::JointTrajectoryPoint jointViaPoint_;
	jointViaPoint_.positions.resize(7);
	jointViaPoint_.velocities.resize(7);
	for (int i = 0; i < 7; ++i) {
		jointViaPoint_.positions[i] = q[i];
		jointViaPoint_.velocities[i] = 0.;
	}
	jointViaPoint_.time_from_start = ros::Duration(agentParams.planTimeOffset);

	state.jointTrajectory.points.clear();
	state.cartTrajectory.points.clear();
	state.jointTrajectory.points.push_back(jointViaPoint_);

	state.jointTrajectory.header.stamp = ros::Time::now();
	state.cartTrajectory.header.stamp = ros::Time::now();
}

Tactic::~Tactic() {

}

void Tactic::updateTactic() {
	if (state.observation.gameStatus.status == START) {
		setNextState();
	} else if (state.observation.gameStatus.status == STOP) {
		setTactic(INIT);
	} else if (state.observation.gameStatus.status == PAUSE) {
		setTactic(HOME);
	}
}

void Tactic::setTactic(Tactics tactic) {
	if (tactic != state.currentTactic) {
		state.isNewTactics = true;
		state.tNewTactics = ros::Time::now().toSec() + agentParams.tTacticSwitchMin;
		ROS_INFO_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Tactics changed: " <<
				tactic2String(state.currentTactic) << " -> " << tactic2String(tactic));
		state.currentTactic = tactic;
	}
}

std::string Tactic::tactic2String(Tactics tactic) {
	switch (tactic) {
		case Tactics::INIT:
			return "INIT   ";
		case Tactics::HOME:
			return "HOME   ";
		case Tactics::SMASH:
			return "SMASH  ";
		case Tactics::CUT:
			return "CUT    ";
		case Tactics::READY:
			return "READY  ";
		case Tactics::REPEL:
			return "REPEL  ";
		case Tactics::PREPARE:
			return "PREPARE";
		default:
			ROS_ERROR_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Invalid Tactic");
			exit(-1);
	}
}

bool Tactic::canSmash() {
	if (state.observation.puckPredictedState.state.x() < agentParams.hitRange.mean() &&
	    state.observation.puckPredictedState.state.x() > agentParams.hitRange[0] &&
	    abs(state.observation.puckPredictedState.state.y()) < (envParams.tableWidth / 2 - envParams.puckRadius - 0.1)) {
		return true;
	}
	if (state.observation.puckPredictedState.state.x() > agentParams.hitRange.mean() &&
		state.observation.puckPredictedState.state.x() < agentParams.hitRange[1] &&
	    state.observation.puckPredictedState.state.dx() < 0) {
		return true;
	}
	return false;
}

bool Tactic::shouldCut() {
	if (state.observation.puckPredictedState.predictedTime < agentParams.tPredictionMax &&
	    (state.observation.puckPredictedState.numOfCollisions > 0 ||
	     (abs(state.observation.puckPredictedState.state.y()) <
	      (envParams.tableWidth / 2 - envParams.malletRadius)))) {
		return true;
	}
	return false;
}

bool Tactic::shouldRepel() {
	if ((state.observation.puckPredictedState.numOfCollisions == 0) &&
	    (state.observation.puckPredictedState.predictedTime < agentParams.tPredictionMax &&
	     state.observation.puckPredictedState.predictedTime > agentParams.tPredictionMax / 2 &&
	     abs(state.observation.puckPredictedState.state.y()) < agentParams.defendZoneWidth / 2)) {
		return true;
	}
	return false;
}

bool Tactic::puckStuck() {
	if (state.isPuckStatic()) {
		if ((state.observation.puckPredictedState.state.x() < agentParams.hitRange[0] &&
		     abs(state.observation.puckPredictedState.state.y()) <
		     (envParams.tableWidth / 2 - 2 * envParams.puckRadius))) {
			return true;
		}
		if (state.observation.puckPredictedState.state.x() < agentParams.hitRange[1] &&
			state.observation.puckPredictedState.state.dx() < 0.05 &&
		    abs(state.observation.puckPredictedState.state.y()) > (envParams.tableWidth / 2 - envParams.puckRadius - 0.1)) {
			return true;
		}
	}
	return false;
}


