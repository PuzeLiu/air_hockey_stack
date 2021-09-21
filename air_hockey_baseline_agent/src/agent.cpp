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


Agent::Agent(ros::NodeHandle nh) :
		nh(nh), state(nh.getNamespace()), rate(100) {
	double r;
	if (nh.getParam("/air_hockey/agent/rate", r)){
		rate = ros::Rate(r);
	}

	loadEnvironmentParams();
	loadAgentParam();

	std::string controllerName = getControllerName();
	observer = new Observer(nh, controllerName, agentParams.defendLine, agentParams.nq);
	agentParams.tPredictionMax = observer->getMaxPredictionTime();

	generator = new TrajectoryGenerator(nh, agentParams, envParams);

	computeBaseConfigurations();

	jointTrajectoryPub = nh.advertise<JointTrajectory>(controllerName + "/command", 2);
	cartTrajectoryPub = nh.advertise<MultiDOFJointTrajectory>("cartesian_trajectory", 1);

	loadTactics();
}

Agent::~Agent() {
	delete observer;
	delete generator;

	for (auto tactic: tacticsProcessor) {
		delete tactic;
	}
}

void Agent::start() {
	ros::Duration(2.).sleep();
	ROS_INFO_STREAM_NAMED(nh.getNamespace(), nh.getNamespace() + ": " + "Agent Started");
	observer->start();

	while (ros::ok()) {
		state.updateObservationAndState(observer->getObservation(), agentParams);

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
}

void Agent::loadAgentParam() {
	agentParams.name = nh.getNamespace();

	string parent_dir = ros::package::getPath("air_hockey_description");
	string robot_description = parent_dir + "/urdf/iiwa_striker_extension_only.urdf";
	pinocchio::urdf::buildModel(robot_description, agentParams.pinoModel);
	agentParams.pinoData = pinocchio::Data(agentParams.pinoModel);
	agentParams.pinoFrameId = agentParams.pinoModel.getFrameId("F_striker_tip");
	agentParams.nq = agentParams.pinoModel.nq;

	std::vector<double> xTmp;

	if (!nh.getParam("/air_hockey/agent/q_ref", xTmp)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/q_ref");
	}
	agentParams.qRef = JointArrayType::Map(xTmp.data(), xTmp.size());

	if (!nh.getParam("/air_hockey/agent/goal", xTmp)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/goal");
	}
	agentParams.xGoal << xTmp[0], xTmp[1];

	if (!nh.getParam("/air_hockey/agent/home", xTmp)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/home");
	}
	agentParams.xHome << xTmp[0], xTmp[1], 0.0;

	if (!nh.getParam("/air_hockey/agent/prepare", xTmp)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/prepare");
	}
	agentParams.xPrepare << xTmp[0], xTmp[1], envParams.universalJointHeight + envParams.puckHeight;

	if (!nh.getParam("/air_hockey/agent/hit_range", xTmp)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/hit_range");
	}
	agentParams.hitRange << xTmp[0], xTmp[1];

	if (!nh.getParam("/air_hockey/agent/min_defend_velocity", agentParams.vDefendMin)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/min_defend_velocity");
	}

	if (!nh.getParam("/air_hockey/agent/min_defend_time", agentParams.tDefendMin)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/min_defend_time");
	}

	if (!nh.getParam("/air_hockey/agent/defend_zone_width", agentParams.defendZoneWidth)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/defend_zone_width");
	}

	if (!nh.getParam("/air_hockey/agent/defend_line", agentParams.defendLine)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/defend_line");
	}

	if (!nh.getParam("/air_hockey/agent/plan_time_offset", agentParams.planTimeOffset)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/plan_time_offset");
	}

	if (!nh.getParam("/air_hockey/agent/min_tactic_switch_time", agentParams.tTacticSwitchMin)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/min_tactic_switch_time");
	}

	if (!nh.getParam("/air_hockey/agent/hit_velocity_scale", agentParams.hitVelocityScale)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/hit_velocity_scale");
	}

	if (!nh.getParam("/air_hockey/agent/init_position_height", agentParams.initHeight)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/init_position_height");
	}
}

void Agent::computeBaseConfigurations() {
	auto transformations = generator->transformations;
	auto optimizer = generator->optimizer;

	// Compute global configuration
	Vector3d xTmp, gc;
	Quaterniond quatTmp;

	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, agentParams.qRef);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	xTmp << agentParams.xHome[0], agentParams.xHome[1], envParams.universalJointHeight;
	transformations->applyForwardTransform(xTmp);

	pinocchio::SE3 oMhome(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
	if (!inverseKinematics(agentParams, oMhome, agentParams.qRef, agentParams.qHome)) {
		ROS_ERROR_STREAM("Inverse Kinematics fail, unable to find solution for HOME position");
	}

	JointArrayType dqTmp(agentParams.nq);
	optimizer->SolveJoint7(agentParams.qHome, dqTmp);

	// compute qinit
	xTmp << agentParams.xHome[0], agentParams.xHome[1], envParams.universalJointHeight + agentParams.initHeight;
	transformations->applyForwardTransform(xTmp);
	pinocchio::SE3 oMinit(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
	if (!inverseKinematics(agentParams, oMinit, agentParams.qRef, agentParams.qInit)) {
		ROS_ERROR_STREAM("Inverse Kinematics fail, unable to find solution for INIT position");
	}

	optimizer->SolveJoint7(agentParams.qInit, dqTmp);
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
	tacticsProcessor[Tactics::REPEL] = new Repel(envParams, agentParams, state, generator);
	tacticsProcessor[Tactics::SMASH] = new Smash(envParams, agentParams, state,
	                                             generator);
}

std::string Agent::getControllerName() {
	std::string controllerName;
	ros::master::V_TopicInfo topics;
	if (ros::master::getTopics(topics)) {
		for (int i = 0; i < topics.size(); ++i) {
			if (topics[i].name == nh.getNamespace() + "/joint_position_trajectory_controller/state") {
				controllerName = "joint_position_trajectory_controller";
				break;
			} else if (topics[i].name == nh.getNamespace() + "/joint_torque_trajectory_controller/state") {
				controllerName = "joint_torque_trajectory_controller";
				break;
			} else if (topics[i].name == nh.getNamespace() + "/joint_feedforward_trajectory_controller/state") {
				controllerName = "joint_feedforward_trajectory_controller";
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
