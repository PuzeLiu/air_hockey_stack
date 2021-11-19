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

#include "air_hockey_baseline_agent/null_space_optimizer.h"
#include "air_hockey_baseline_agent/utils.h"
#include <iostream>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <chrono>


using namespace std;
using namespace air_hockey_baseline_agent;
using namespace Eigen;

void loadAgentParams(AgentParams &agentParams) {
	agentParams.name = "/iiwa_front";

	string parent_dir = ros::package::getPath("air_hockey_description");
	string robot_description = parent_dir + "/urdf/iiwa_striker_extension_only.urdf";
	pinocchio::urdf::buildModel(robot_description, agentParams.pinoModel);
	agentParams.pinoData = pinocchio::Data(agentParams.pinoModel);
	agentParams.pinoFrameId = agentParams.pinoModel.getFrameId("F_striker_tip");
	agentParams.nq = agentParams.pinoModel.nq;

	agentParams.qRef.resize(agentParams.pinoModel.nq);
	agentParams.qRef << 0., 0.15378149, 0., -1.3630843, 0., 1.3767066, 0.;
	agentParams.xGoal << 1.98, 0.0;
	agentParams.xHome << 0.08, 0.0, 0.0;
	agentParams.xPrepare << 0.4, 0.0, 0.0;

	agentParams.hitRange << 0.2, 0.9;

	agentParams.vDefendMin = 0.08;
	agentParams.tDefendMin = 0.3;
	agentParams.defendZoneWidth = 0.4;
	agentParams.defendLine = 0.2;
	agentParams.planTimeOffset = 0.1;
	agentParams.tPredictionMax = 1.5;
	agentParams.tTacticSwitchMin = 1.5;
	agentParams.hitVelocityScale = 1.0;
	agentParams.initHeight = 0.2;
}

void initOptimizerData(OptimizerData &optData) {
	optData.rate = 100.;
	optData.K.setConstant(optData.rate);
	optData.weights << 10., 10., 5., 10., 1., 1., 0.;
	optData.weightsAnchor << 1., 1., 5., 1, 10., 10., 0.;
}

trajectory_msgs::MultiDOFJointTrajectory readCartesianTrajectory() {
	rosbag::Bag bag;

	string parent_dir = ros::package::getPath("air_hockey_baseline_agent");
	string bag_file = parent_dir + "/examples/cartesian_hit.bag";
	bag.open(bag_file, rosbag::BagMode::Read);
	rosbag::View view(bag, rosbag::TopicQuery("/iiwa_front/cartesian_trajectory"));
	return *view.begin()->instantiate<trajectory_msgs::MultiDOFJointTrajectory>().get();
}

int main(int argc, char *argv[]) {
	AgentParams agentParams;
	loadAgentParams(agentParams);

	OptimizerData optData(agentParams);
	initOptimizerData(optData);

	NullSpaceOptimizer optimizer(agentParams, optData);

	trajectory_msgs::MultiDOFJointTrajectory cartTraj = readCartesianTrajectory();

	trajectory_msgs::JointTrajectory jointTraj;

	JointArrayType qStart(agentParams.nq), qAnchor(agentParams.nq);

	// Set up start configuration
	Eigen::Vector3d xDes;
	xDes << cartTraj.points[0].transforms[0].translation.x,
			cartTraj.points[0].transforms[0].translation.y,
			cartTraj.points[0].transforms[0].translation.z;
	Eigen::VectorXd qInit(7);
	qInit << 0., 0.15378149,  0., -1.3630843, 0.,  1.3767066, 0.;
	air_hockey_baseline_agent::inverseKinematicsPosition(agentParams, xDes, qInit, qStart);

	bool success;

	/** Original QP Optimization*/
	cout << "#################################" << endl;
	cout << "#     Test QP Optimization      #" << endl;
	cout << "#################################" << endl;
	auto start = chrono::high_resolution_clock::now();
	for (int j = 0; j < 100; ++j) {
		success = optimizer.optimizeJointTrajectory(cartTraj, qStart, jointTraj);
	}
	auto finish = chrono::high_resolution_clock::now();
	cout << "Number of Points: " << cartTraj.points.size() << ", Success: " << success << ", Optimization Time: "
	     << chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() / 100 / 1.e6 << "ms\n";


	/** AQP Optimization*/
	cout << "#################################" << endl;
	cout << "#    Test AQP Optimization      #" << endl;
	cout << "#################################" << endl;
	qAnchor.setZero();
	jointTraj.points.clear();
	start = chrono::high_resolution_clock::now();
	for (int j = 0; j < 100; ++j) {
		success = optimizer.optimizeJointTrajectoryAnchor(cartTraj, qStart, qAnchor, jointTraj, true);
	}
	finish = chrono::high_resolution_clock::now();
	cout << "Number of Points: " << cartTraj.points.size() << ", Success: " << success << ", Optimization Time: "
	     << chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() / 100 / 1.e6 << "ms\n";
	return 0;
}
