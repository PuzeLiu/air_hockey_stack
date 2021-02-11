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

#include "air_hockey_baseline_agent/agent.h"
#include "air_hockey_baseline_agent/tactics.h"

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace air_hockey_baseline_agent;
using namespace trajectory_msgs;
using namespace Eigen;

AgentState::AgentState() {
	currentTactic = Tactics::READY;
	gameStatusPrev = GameStatus::STOP;
	smashCount = 0;
	cutCount = 0;
	cutPrevY = 0.0;
	staticCount = 0;
}

Agent::Agent(ros::NodeHandle nh, double rate) :
		nh(nh), rate(rate) {

	loadEnvironmentParams();
	loadAgentParam();

	std::string controllerName = getControllerName();
	observer = new Observer(nh, controllerName, agentParams.defendLine_);

	generator = new TrajectoryGenerator(nh.getNamespace(), envParams, observer, rate);


	jointTrajectoryPub = nh.advertise<JointTrajectory>(
			controllerName + "/command", 2);
	cartTrajectoryPub = nh.advertise<MultiDOFJointTrajectory>(
			"cartesian_trajectory", 1);

	loadTactics();

	state.trajStopTime_ = ros::Time::now();
}

Agent::~Agent() {
	delete observer;
	delete generator;

	for (auto tactic : tactics) {
		delete tactic;
	}
}

void Agent::start() {
	ros::Duration(2.).sleep();
	ROS_INFO_STREAM("Agent Started");
	observer->start();

	while (ros::ok()) {
		agentState.observation = observer->getObservation();
		auto &activeTactic = *tactics[agentState.currentTactic];
		if (activeTactic.ready()) {
			activeTactic.apply();
			publishTrajectory();
		}

		update();
		rate.sleep();
	}
}

void Agent::update() {
	bool gameStatusChanged = (agentState.observation.gameStatus.status
			!= agentState.gameStatusPrev);
	if (gameStatusChanged) {
		agentState.gameStatusPrev =
				static_cast<GameStatus>(agentState.observation.gameStatus.status);
		if (agentState.observation.gameStatus.status == GameStatus::START) {
			ROS_INFO_STREAM("Game Status Changed: START");
			agentState.currentTactic = Tactics::HOME;
		} else if (agentState.observation.gameStatus.status
				== GameStatus::PAUSE) {
			ROS_INFO_STREAM("Game Status Changed: PAUSE");
			agentState.currentTactic = Tactics::READY;
		} else {
			ROS_INFO_STREAM("Game Status Changed: STOP");
			agentState.currentTactic = Tactics::INIT;
		}
	} else {
		if (agentState.observation.gameStatus.status == GameStatus::START) {
			updateTactic();
		}
	}
}

void Agent::updateTactic() {
	if (agentState.observation.puckPredictedState.state.block<2, 1>(2, 0).norm()
			> agentParams.vDefendMin_) {
		agentState.staticCount = 0;
	} else {
		agentState.staticCount += 1;
	}

	if (agentState.staticCount < 10) {
		if (agentState.observation.puckPredictedState.predictedTime
				< observer->getMaxPredictionTime()) {
			setTactic(Tactics::CUT);
			if (abs(
					agentState.observation.puckPredictedState.state.y()
							- agentState.cutPrevY)
					> (envParams.malletRadius_)) {
				agentState.cutCount = 0;
				agentState.cutPrevY =
						agentState.observation.puckPredictedState.state.y();
			} else {
				agentState.cutCount += 1;
			}
			if (agentState.cutCount == 10) {
				agentState.currentTactic = Tactics::CUT;
			}
		} else {
			bool restart = setTactic(Tactics::READY);
			agentState.currentTactic = Tactics::READY;
		}
	} else {
		if (agentState.observation.puckPredictedState.state.x()
				< agentParams.hitRange_[1]
				&& agentState.observation.puckPredictedState.state.x()
						> agentParams.hitRange_[0]
				&& agentState.currentTactic != Tactics::PREPARE) {
			if (agentState.observation.puckPredictedState.state.y()
					> envParams.tableWidth_ / 2 - envParams.puckRadius_
							- envParams.malletRadius_ - 0.02
					|| agentState.observation.puckPredictedState.state.y()
							< -envParams.tableWidth_ / 2 + envParams.puckRadius_
									+ envParams.malletRadius_ + 0.02) {
				bool restart = setTactic(Tactics::PREPARE);
				agentState.currentTactic = Tactics::PREPARE;
			} else {
				bool restart = setTactic(Tactics::SMASH);
				if (agentState.smashCount > 10) {
					agentState.currentTactic = Tactics::SMASH;
				}
			}
		} else if (agentState.observation.puckPredictedState.state.x()
				<= agentParams.hitRange_[0]
				|| agentState.currentTactic == Tactics::PREPARE) {
			bool restart = setTactic(Tactics::PREPARE);
			agentState.currentTactic = Tactics::PREPARE;
		} else {
			bool restart = setTactic(Tactics::READY);
			agentState.currentTactic = Tactics::READY;
		}
	}

}

void Agent::publishTrajectory() {
	jointTrajectoryPub.publish(state.jointTrajectory_);
	cartTrajectoryPub.publish(state.cartTrajectory_);
}

void Agent::loadTactics() {
	tactics.resize(Tactics::N_TACTICS);

	tactics[Tactics::INIT] = new Init(envParams, agentParams, state, generator);
	tactics[Tactics::HOME] = new Home(envParams, agentParams, state, generator);
	tactics[Tactics::READY] = new Ready(envParams, agentParams, state, generator);
	tactics[Tactics::PREPARE] = new Prepare(envParams, agentParams, state, generator);
	tactics[Tactics::CUT] = new Cut(envParams, agentParams, state, generator);
	tactics[Tactics::SMASH] = new Smash(envParams, agentParams, state, generator);
}

bool Agent::setTactic(Tactics tactic) {
//	if (tacticState_ != tactic) {
//		if (tactic == Tactics::READY) {
//			if (tacticState_ == Tactics::SMASH
//					&& ros::Time::now() < trajStopTime_) {
//				return false;
//			} else {
//				tacticState_ = tactic;
//				ROS_INFO_STREAM("Tactics changed: READY");
//				return true;
//			}
//		} else if (tactic == Tactics::CUT) {
//			tacticState_ = tactic;
//			ROS_INFO_STREAM("Tactics changed: CUT");
//			return true;
//		} else if (tactic == Tactics::PREPARE) {
//			tacticState_ = tactic;
//			ROS_INFO_STREAM("Tactics changed: PREPARE");
//			return true;
//		} else if (tactic == Tactics::SMASH) {
//			tacticState_ = tactic;
//			smashCount = 0;
//			ROS_INFO_STREAM("Tactics changed: SMASH");
//			return true;
//		}
//	} else if (tactic == Tactics::SMASH) {
//		smashCount += 1;
//		return false;
//	}
	return false;
}

std::string Agent::getControllerName() {
	std::string controllerName;
	ros::master::V_TopicInfo topics;
	if (ros::master::getTopics(topics)) {
		for (int i = 0; i < topics.size(); ++i) {
			if (topics[i].name
					== nh.getNamespace()
							+ "/joint_position_trajectory_controller/state") {
				controllerName = "joint_position_trajectory_controller";
				break;
			} else if (topics[i].name
					== nh.getNamespace()
							+ "/joint_torque_trajectory_controller/state") {
				controllerName = "joint_torque_trajectory_controller";
				break;
			}
		}

		if (controllerName == "") {
			ROS_FATAL_STREAM("Could not find controller");
			exit(-1);
		}
	}

	return controllerName;
}

void Agent::loadEnvironmentParams() {
	nh.getParam("/air_hockey/table_length", envParams.tableLength_);
	nh.getParam("/air_hockey/table_width", envParams.tableWidth_);
	nh.getParam("/air_hockey/mallet_radius", envParams.malletRadius_);
	nh.getParam("/air_hockey/puck_radius", envParams.puckRadius_);
	nh.getParam("/air_hockey/puck_height", envParams.puckHeight_);
	nh.getParam("/air_hockey/agent/universal_joint_height",
			envParams.universalJointHeight_);
	nh.getParam("/air_hockey/agent/prepare_height", envParams.prepareHeight_);

	std::vector<double> tcp_position;
	nh.getParam("/air_hockey/agent/tcp_position", tcp_position);
	envParams.tcp_position.x() = tcp_position[0];
	envParams.tcp_position.y() = tcp_position[1];
	envParams.tcp_position.z() = tcp_position[2];

	std::vector<double> tcp_quaternion;
	nh.getParam("/air_hockey/agent/tcp_quaternion", tcp_quaternion);
	envParams.tcp_quaternion.w() = tcp_quaternion[0];
	envParams.tcp_quaternion.x() = tcp_quaternion[1];
	envParams.tcp_quaternion.y() = tcp_quaternion[2];
	envParams.tcp_quaternion.z() = tcp_quaternion[3];
}

void Agent::loadAgentParam() {
	std::vector<double> xTmp;

	nh.getParam("/air_hockey/agent/home", xTmp);
	agentParams.xHome_ << xTmp[0], xTmp[1];

	nh.getParam("/air_hockey/agent/prepare", xTmp);
	agentParams.xPrepare_ << xTmp[0], xTmp[1], envParams.universalJointHeight_
			+ envParams.puckHeight_;

	nh.getParam("/air_hockey/agent/goal", xTmp);
	agentParams.xGoal_ << xTmp[0], xTmp[1];

	nh.getParam("/air_hockey/agent/hit_range", xTmp);
	agentParams.hitRange_ << xTmp[0], xTmp[1];

	nh.getParam("/air_hockey/agent/q_ref", xTmp);
	agentParams.qRef_ << xTmp[0], xTmp[1], xTmp[2], xTmp[3], xTmp[4], xTmp[5], xTmp[6];

	nh.getParam("/air_hockey/agent/defend_line", agentParams.defendLine_);

	nh.getParam("/air_hockey/agent/max_hit_velocity", agentParams.vHitMax_);

	nh.getParam("/air_hockey/agent/min_defend_velocity",
			agentParams.vDefendMin_);

	nh.getParam("/air_hockey/agent/min_defend_time", agentParams.tDefendMin_);
	nh.getParam("/air_hockey/agent/plan_time_offset",
			agentParams.planTimeOffset_);
}

