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

using namespace air_hockey_baseline_agent;
using namespace trajectory_msgs;
using namespace Eigen;


Agent::Agent(ros::NodeHandle nh, double rate) :
		nh(nh), rate(rate), state(nh.getNamespace()) {

	loadEnvironmentParams();
	loadAgentParam();

	std::string controllerName = getControllerName();
	observer = new Observer(nh, controllerName, agentParams.defendLine);
	agentParams.maxPredictionTime = observer->getMaxPredictionTime();

	generator = new TrajectoryGenerator(nh.getNamespace(), envParams, observer,
	                                    rate);

	computeBaseConfigurations();

	jointTrajectoryPub = nh.advertise<JointTrajectory>(
			controllerName + "/command", 2);
	cartTrajectoryPub = nh.advertise<MultiDOFJointTrajectory>(
			"cartesian_trajectory", 1);

	loadTactics();
}

Agent::~Agent() {
	delete observer;
	delete generator;

	for (auto tactic : tacticsProcessor) {
		delete tactic;
	}
}

void Agent::start() {
	ros::Duration(2.).sleep();
	ROS_INFO_STREAM("Agent Started");
	observer->start();

	while (ros::ok()) {
		state.updateObservationAndState(observer->getObservation(), agentParams);
//		update();
		tacticsProcessor[state.currentTactic]->updateTactic();

		auto &activeTactic = *tacticsProcessor[state.currentTactic];
		if (activeTactic.ready()) {
			if (activeTactic.apply()) {
				publishTrajectory();
			}
		}
		rate.sleep();
	}
}

void Agent::update() {
	auto status = state.observation.gameStatus.status;
	if (state.isNewTactics) {
		switch (status) {
			case GameStatus::START:
				ROS_INFO_STREAM("Game Status Changed: START");
				state.currentTactic = Tactics::HOME;
				break;
			case GameStatus::STOP:
				ROS_INFO_STREAM("Game Status Changed: STOP");
				state.currentTactic = Tactics::INIT;
				break;
			case GameStatus::PAUSE:
				ROS_INFO_STREAM("Game Status Changed: PAUSE");
				state.currentTactic = Tactics::READY;
				break;
			default:
				ROS_FATAL_STREAM("Unknown game status");
				exit(-1);
		}
	} else if (status == GameStatus::START) {
		tacticsProcessor[state.currentTactic]->updateTactic();
	}
}

void Agent::publishTrajectory() {
	jointTrajectoryPub.publish(state.jointTrajectory);
	cartTrajectoryPub.publish(state.cartTrajectory);
}

void Agent::loadEnvironmentParams() {
	nh.getParam("/air_hockey/table_length", envParams.tableLength);
	nh.getParam("/air_hockey/table_width", envParams.tableWidth);
	nh.getParam("/air_hockey/mallet_radius", envParams.malletRadius);
	nh.getParam("/air_hockey/puck_radius", envParams.puckRadius);
	nh.getParam("/air_hockey/puck_height", envParams.puckHeight);
	nh.getParam("/air_hockey/agent/universal_joint_height",
	            envParams.universalJointHeight);
	nh.getParam("/air_hockey/agent/prepare_height", envParams.prepareHeight);

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
	agentParams.xHome << xTmp[0], xTmp[1], 0.0;

	nh.getParam("/air_hockey/agent/prepare", xTmp);
	agentParams.xPrepare << xTmp[0], xTmp[1], envParams.universalJointHeight
	                                          + envParams.puckHeight;

	nh.getParam("/air_hockey/agent/goal", xTmp);
	agentParams.xGoal << xTmp[0], xTmp[1];

	nh.getParam("/air_hockey/agent/hit_range", xTmp);
	agentParams.hitRange << xTmp[0], xTmp[1];

	nh.getParam("/air_hockey/agent/q_ref", xTmp);
	agentParams.qRef << xTmp[0], xTmp[1], xTmp[2], xTmp[3], xTmp[4], xTmp[5], xTmp[6];

	nh.getParam("/air_hockey/agent/defend_line", agentParams.defendLine);

	nh.getParam("/air_hockey/agent/max_hit_velocity", agentParams.vHitMax);

	nh.getParam("/air_hockey/agent/min_defend_velocity",
	            agentParams.vDefendMin);

	nh.getParam("/air_hockey/agent/min_defend_time", agentParams.tDefendMin);
	nh.getParam("/air_hockey/agent/plan_time_offset",
	            agentParams.planTimeOffset);

	envParams.initHeight = 0.2; //TODO in config
}

void Agent::computeBaseConfigurations() {

	auto kinematics = generator->kinematics;
	auto transformations = generator->transformations;
	auto optimizer = generator->optimizer;

	// Compute global configuration
	Vector3d xTmp, gc;
	Quaterniond quatTmp;
	double psi;

	kinematics->forwardKinematics(agentParams.qRef, xTmp, quatTmp);
	kinematics->getRedundancy(agentParams.qRef, gc, psi);

	// compute qHome
	xTmp << agentParams.xHome[0], agentParams.xHome[1], envParams.universalJointHeight;
	transformations->applyForwardTransform(xTmp);
	if (!kinematics->inverseKinematics(xTmp, quatTmp, gc, psi,
	                                   agentParams.qHome)) {
		ROS_ERROR_STREAM(
				"Inverse Kinematics fail, unable to find solution for HOME position");
	}

	optimizer->SolveJoint7(agentParams.qHome);

	// compute qinit
	xTmp << agentParams.xHome[0], agentParams.xHome[1], envParams.universalJointHeight
	                                                    + envParams.initHeight;
	transformations->applyForwardTransform(xTmp);
	if (!kinematics->inverseKinematics(xTmp, quatTmp, gc, psi,
	                                   agentParams.qInit)) {
		ROS_ERROR_STREAM(
				"Inverse Kinematics fail, unable to find solution for INIT position");
	}

	optimizer->SolveJoint7(agentParams.qInit);
}

void Agent::loadTactics() {
	tacticsProcessor.resize(Tactics::N_TACTICS);

	tacticsProcessor[Tactics::INIT] = new Init(envParams, agentParams, state, generator);
	tacticsProcessor[Tactics::HOME] = new Home(envParams, agentParams, state, generator);
	tacticsProcessor[Tactics::READY] = new Ready(envParams, agentParams, state,
	                                             generator);
	tacticsProcessor[Tactics::PREPARE] = new Prepare(envParams, agentParams, state,
	                                                 generator);
	tacticsProcessor[Tactics::CUT] = new Cut(envParams, agentParams, state, generator);
	tacticsProcessor[Tactics::SMASH] = new Smash(envParams, agentParams, state,
	                                             generator);
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
