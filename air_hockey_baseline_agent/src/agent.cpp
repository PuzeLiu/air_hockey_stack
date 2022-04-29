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

#include "air_hockey_baseline_agent/agent.h"

using namespace air_hockey_baseline_agent;
using namespace trajectory_msgs;
using namespace Eigen;


Agent::Agent(ros::NodeHandle nh) :
        nh(nh), state(nh.getNamespace()), rate(100) {
    double r;
    if (nh.getParam("/air_hockey/agent/rate", r)) {
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

    if (agentParams.debugTactics) {
        ROS_INFO_STREAM_NAMED(nh.getNamespace(), nh.getNamespace() + ": " + "Debugging Mode");
        tacticService = nh.advertiseService("set_tactic", &Agent::setTacticService, this);
    }

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
//    nh.getParam("/air_hockey/agent/prepare_height", envParams.prepareHeight);
}

void Agent::loadAgentParam() {
    agentParams.name = nh.getNamespace();

    string joint_prefix;
    if (agentParams.name == "/iiwa_front") joint_prefix = "F";
    else if (agentParams.name == "/iiwa_back") joint_prefix = "B";

    string robot_description;
    nh.getParam("iiwa_only_description", robot_description);
    pinocchio::urdf::buildModelFromXML(robot_description, agentParams.pinoModel);

    std::vector<double> tcp_position, tcp_quaternion;
    nh.getParam("/air_hockey/agent/tcp_position", tcp_position);
    nh.getParam("/air_hockey/agent/tcp_quaternion", tcp_quaternion);
    Eigen::Vector3d tcp_pos_eig(tcp_position[0], tcp_position[1], tcp_position[2]);
    Eigen::Quaterniond tcp_quat_eig(tcp_quaternion[0], tcp_quaternion[1], tcp_quaternion[2], tcp_quaternion[3]);
    pinocchio::SE3 tip_frame(tcp_quat_eig, tcp_pos_eig);
    pinocchio::Frame striker_tip_frame(joint_prefix +"_striker_tip",
                                       agentParams.pinoModel.getJointId(joint_prefix +"_joint_7"),
                                       agentParams.pinoModel.nframes - 1,
                                       tip_frame, pinocchio::OP_FRAME);

    agentParams.pinoModel.addFrame(striker_tip_frame, false);

    agentParams.pinoData = pinocchio::Data(agentParams.pinoModel);
    agentParams.pinoFrameId = agentParams.pinoModel.getFrameId(joint_prefix + "_striker_tip");
    agentParams.nq = agentParams.pinoModel.nq;

    std::vector<double> xTmp;
    nh.param("/air_hockey/agent/debug_tactics", agentParams.debugTactics, false);

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
    agentParams.xPrepare << xTmp[0], xTmp[1], agentParams.universalJointHeight + envParams.puckHeight;

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

	if (!nh.getParam("/air_hockey/agent/universal_joint_height", agentParams.universalJointHeight)) {
		ROS_ERROR_STREAM("Unable to find /air_hockey/agent/universal_joint_height");
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
    xTmp << agentParams.xHome[0], agentParams.xHome[1], agentParams.universalJointHeight;
	transformations->transformTable2Robot(xTmp);

    pinocchio::SE3 oMhome(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
    if (!inverseKinematics(agentParams, oMhome, agentParams.qRef, agentParams.qHome)) {
        ROS_ERROR_STREAM("Inverse Kinematics fail, unable to find solution for HOME position");
    }

    JointArrayType dqTmp(agentParams.nq);
    optimizer->solveJoint7(agentParams.qHome, dqTmp);

    // compute qinit
    xTmp << agentParams.xHome[0], agentParams.xHome[1], agentParams.universalJointHeight + agentParams.initHeight;
	transformations->transformTable2Robot(xTmp);
    pinocchio::SE3 oMinit(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
    if (!inverseKinematics(agentParams, oMinit, agentParams.qRef, agentParams.qInit)) {
        ROS_ERROR_STREAM("Inverse Kinematics fail, unable to find solution for INIT position");
    }

    optimizer->solveJoint7(agentParams.qInit, dqTmp);
}

void Agent::loadTactics() {
    tacticsProcessor.resize(Tactics::N_TACTICS);

    tacticsProcessor[Tactics::INIT] = new Init(envParams, agentParams, state, generator);
    tacticsProcessor[Tactics::HOME] = new Home(envParams, agentParams, state, generator);
	tacticsProcessor[Tactics::READY] = new Ready(envParams, agentParams, state, generator);
	tacticsProcessor[Tactics::PREPARE] = new Prepare(envParams, agentParams, state, generator);
    tacticsProcessor[Tactics::CUT] = new Cut(envParams, agentParams, state, generator);
    tacticsProcessor[Tactics::REPEL] = new Repel(envParams, agentParams, state, generator);
	tacticsProcessor[Tactics::SMASH] = new Smash(envParams, agentParams, state, generator);
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
			} else if (topics[i].name == nh.getNamespace() + "/adrc_trajectory_controller/state") {
				controllerName = "adrc_trajectory_controller";
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

bool Agent::setTacticService(SetTacticsService::Request &req, SetTacticsService::Response &res) {
    if (req.tactic == "SMASH") {
        agentParams.smashStrategy = req.smashStrategy;
        ROS_INFO_STREAM("Set Tactic Service with smash Strategy: " << req.smashStrategy);
        tacticsProcessor[state.currentTactic]->setTactic(Tactics::SMASH);
    } else if (req.tactic == "CUT") {
        int count = 0;
        while (count < 120) {
            if (tacticsProcessor[state.currentTactic]->shouldCut()) {
                tacticsProcessor[state.currentTactic]->setTactic(Tactics::CUT);
                break;
            }
            count++;
            rate.sleep();
        }
    } else if (req.tactic == "REPEL") {
        int count = 0;
        while (count < 120) {
            if (tacticsProcessor[state.currentTactic]->shouldRepel()) {
                tacticsProcessor[state.currentTactic]->setTactic(Tactics::REPEL);
                break;
            }
            count++;
            rate.sleep();
        }
    } else if (req.tactic == "PREPARE") {
        tacticsProcessor[state.currentTactic]->setTactic(Tactics::PREPARE);
    } else {
        return false;
    }

    res.success = true;
    return true;
}