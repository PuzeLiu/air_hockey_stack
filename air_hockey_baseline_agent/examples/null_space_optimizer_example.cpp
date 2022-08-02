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
#include "air_hockey_baseline_agent/hitting_point_optimizer.h"
#include "air_hockey_baseline_agent/planner/combinatorial_hit_new.h"
#include "air_hockey_baseline_agent/planner/cubic_linear_motion.h"
#include "air_hockey_baseline_agent/transformations.h"
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

Transformations setTableRobotTransformation()
{
	// Construct Transform from Table to Robot
	tf2::Stamped<tf2::Transform> tfTable2Robot;
	tfTable2Robot.setOrigin(tf2::Vector3(0.526, 0, 0.1));
	tfTable2Robot.setRotation(tf2::Quaternion(0., 0., 0., 1.));
	geometry_msgs::TransformStamped tfTable2RobotMsg = tf2::toMsg(tfTable2Robot);
	return Transformations(tfTable2RobotMsg);
}

void loadEnvParams(EnvironmentParams& envParams)
{
	envParams.malletRadius = 0.04815;
	envParams.puckRadius = 0.03165;
	envParams.puckHeight = 0.006;
	envParams.tableLength = 1.956;
	envParams.tableWidth = 1.042;
}

void loadAgentParams(AgentParams& agentParams, Transformations transformations)
{
	agentParams.name = "/iiwa_front";
	agentParams.debugTactics = false;
	agentParams.rate = 100.;

	string parent_dir = ros::package::getPath("air_hockey_description");
	string robot_description = parent_dir + "/urdf/iiwa_striker.urdf";
	pinocchio::urdf::buildModel(robot_description, agentParams.pinoModel);
	agentParams.pinoData = pinocchio::Data(agentParams.pinoModel);
	agentParams.pinoFrameId = agentParams.pinoModel.getFrameId("F_striker_tip");
	agentParams.nq = agentParams.pinoModel.nq;

	agentParams.qRef.resize(agentParams.pinoModel.nq);
	agentParams.qRef.block<7, 1>(0, 0) << 0., 0.06580, 0., -1.45996, 0., 1.22487, 0.;
	agentParams.xGoal << 1.98, 0.0;
	agentParams.xHome << 0.08, 0.0, 0.0;
	agentParams.xInit << 0.4, 0.0, 0.0;

	agentParams.hitRange << 0.2, 0.8;
	agentParams.eeMaxAcceleration = 8.0;

	agentParams.staticVelocityThreshold = 0.08;
	agentParams.defendMinTime = 0.3;
	agentParams.defendZoneWidth = 0.4;
	agentParams.defendLine = 0.2;
	agentParams.planTimeOffset = 0.1;
	agentParams.tPredictionMax = 1.5;
	agentParams.tTacticSwitchMin = 1.5;
	agentParams.hitVelocityScale = 1.0;
	agentParams.initHeight = 0.2;
	agentParams.universalJointHeight = 0.0505;  // Table Height: 0.0505 + 0.1, TODO change to 0.0645 + 0.1
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, agentParams.qRef);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);

	// Construct qHome
	Vector3d xTmp;
	xTmp << agentParams.xHome[0], agentParams.xHome[1], agentParams.universalJointHeight;
	transformations.transformTable2Robot(xTmp);
	pinocchio::SE3 oMhome(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
	air_hockey_baseline_agent::inverseKinematics(agentParams, oMhome, agentParams.qRef, agentParams.qHome);
	// Construct qInit
	xTmp << agentParams.xHome[0], agentParams.xHome[1], agentParams.universalJointHeight + agentParams.initHeight;
	transformations.transformTable2Robot(xTmp);
	pinocchio::SE3 oMinit(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
	air_hockey_baseline_agent::inverseKinematics(agentParams, oMinit, agentParams.qRef, agentParams.qInit);
}

void initOptimizerData(OptimizerData& optData)
{
	optData.K = 100.;
	optData.weights << 10., 10., 5., 10., 1., 1., 0.;
	optData.weightsAnchor << 1., 1., 5., 1, 10., 10., 0.;
}

void getStopPoint(const Vector2d &hitPoint, const AgentParams &agentParams,
	const EnvironmentParams &envParams, Vector2d &stopPoint) {
	if (hitPoint.y() > 0) {
		stopPoint = hitPoint + Vector2d(0.2, -0.25);
	} else {
		stopPoint = hitPoint + Vector2d(0.2, 0.25);
	}

	stopPoint.x() = fmin(stopPoint.x(), agentParams.hitRange[1]);
	stopPoint.y() = fmax(fmin(stopPoint.y(), envParams.tableWidth / 2 - envParams.malletRadius - 0.05),
		-envParams.tableWidth / 2 + envParams.malletRadius + 0.05);
}

int main(int argc, char* argv[])
{
	ros::Time::init();
	rosbag::Bag bag;
	std::string log_path = ros::package::getPath("air_hockey_baseline_agent");

	srand((unsigned int) time(0));

	EnvironmentParams envParams{};
	loadEnvParams(envParams);

	Transformations transformations = setTableRobotTransformation();

	AgentParams agentParams;
	loadAgentParams(agentParams, transformations);

	OptimizerData optData(agentParams);
	initOptimizerData(optData);

	NullSpaceOptimizer nullOptimizer(agentParams, optData);
	HittingPointOptimizer hittingPointOptimizer(agentParams, optData);

	// Construct Planner
	Vector2d bound_lower(envParams.malletRadius + 0.02,
		-envParams.tableWidth / 2 + envParams.malletRadius + 0.02);
	Vector2d bound_upper(envParams.tableLength / 2 - envParams.malletRadius,
		envParams.tableWidth / 2 - envParams.malletRadius - 0.02);
	CombinatorialHitNew combPlanner(bound_lower, bound_upper, agentParams.rate,
		agentParams.universalJointHeight, agentParams.eeMaxAcceleration);
	CubicLinearMotion cubPlanner(agentParams.rate, agentParams.universalJointHeight);

	trajectory_msgs::MultiDOFJointTrajectory cartTraj;
	cartTraj.joint_names.push_back("x");
	cartTraj.joint_names.push_back("y");
	cartTraj.joint_names.push_back("z");
	trajectory_msgs::JointTrajectory jointTraj;
	jointTraj.joint_names.push_back("F_joint_1");
	jointTraj.joint_names.push_back("F_joint_2");
	jointTraj.joint_names.push_back("F_joint_3");
	jointTraj.joint_names.push_back("F_joint_4");
	jointTraj.joint_names.push_back("F_joint_5");
	jointTraj.joint_names.push_back("F_joint_6");
	jointTraj.joint_names.push_back("F_joint_7");

	bool success;

	// Setup starting and stop point on the table surface
	Vector2d xStart2d, vStart2d, xHit2d, vHit2d, xEnd2d, vEnd2d, xStop2d, vStop2d;
	xStart2d << agentParams.xHome[0], agentParams.xHome[1]; // The start point and qStart in nullOptimizer should match
	vStart2d.setZero();
	xHit2d.setRandom();
	xHit2d[0] = xHit2d[0] * (agentParams.hitRange[1] - agentParams.hitRange[0]) / 2
		+ (agentParams.hitRange[0] + agentParams.hitRange[1]) / 2;
	xHit2d[1] = xHit2d[1] * (envParams.tableWidth / 2 - 0.12 - envParams.malletRadius);
	vHit2d.setRandom();
	vHit2d[0] = (vHit2d[0] + 1)/2;
	vHit2d[1] = vHit2d[1];
	getStopPoint(xHit2d, agentParams, envParams, xStop2d);
	vStop2d.setZero();

	//! Get Maximum Hitting Velocity
	double velMag, hitting_time;
	VectorXd qAnchor(agentParams.nq);
    VectorXd dqStart(agentParams.nq);
	qAnchor.setZero();
	Vector3d hitPoint3d, hitDir3d;
	hitPoint3d.topRows(2) = xHit2d;
	hitPoint3d[2] = agentParams.universalJointHeight;
	hitDir3d.topRows(2) = vHit2d;
	hitDir3d[2] = 0.;
	hitDir3d.normalize();
	transformations.transformTable2Robot(hitPoint3d);
	transformations.rotationTable2Robot(hitDir3d);

	hittingPointOptimizer.solve(hitPoint3d, hitDir3d, qAnchor, velMag);
	hitDir3d = hitDir3d * velMag;
	vHit2d = hitDir3d.topRows(2);

	/** Original QP Optimization*/
	cout << "#################################" << endl;
	cout << "#     Test QP Optimization      #" << endl;
	cout << "#################################" << endl;
	auto start = chrono::high_resolution_clock::now();
	for (int j = 0; j < 20; ++j)
	{
		cartTraj.points.clear();
		jointTraj.points.clear();
		if (not combPlanner.plan(xStart2d, vStart2d, xHit2d, vHit2d, hitting_time, xStop2d, vStop2d, cartTraj)){
			std::cerr << "plan failed" << std::endl;
			break;
		}
		transformations.transformTrajectory(cartTraj);

		JointArrayType dqZero;
		dqZero.setZero();
		if (nullOptimizer.optimizeJointTrajectory(cartTraj, agentParams.qHome, dqZero, jointTraj)) {
			success = true;
			break;
		} else {
			vHit2d = vHit2d * 0.9;
		}
	}
	auto finish = chrono::high_resolution_clock::now();
	cout << "Number of Points: " << cartTraj.points.size() << ", Success: " << success << ", Optimization Time: "
		 << chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() / 1.e6 << "ms\n";

	bag.open(log_path + "/examples/qp_opt.bag", rosbag::bagmode::Write);
	bag.write("cartesian_trajectory", ros::Time::now(), cartTraj);
	bag.write("joint_trajectory", ros::Time::now(), jointTraj);
	bag.close();


	/** AQP Optimization*/
	cout << "#################################" << endl;
	cout << "#    Test AQP Optimization      #" << endl;
	cout << "#################################" << endl;
	success = false;
	start = chrono::high_resolution_clock::now();
	for (int j = 0; j < 20; ++j)
	{
		cartTraj.points.clear();
		jointTraj.points.clear();

		if (not combPlanner.plan(xStart2d, vStart2d, xHit2d, vHit2d, hitting_time, xStop2d, vStop2d, cartTraj)){
			break;
		}
		transformations.transformTrajectory(cartTraj);

        dqStart.setZero();
		if (nullOptimizer.optimizeJointTrajectoryAnchor(cartTraj, agentParams.qHome, dqStart, qAnchor, hitting_time, jointTraj)) {
			success = true;
			break;
		} else {
			vHit2d = vHit2d * 0.9;
			continue;
		}
	}
	finish = chrono::high_resolution_clock::now();
	cout << "Number of Points: " << cartTraj.points.size() << ", Success: " << success << ", Optimization Time: "
		 << chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() / 1.e6 << "ms\n";

	bag.open(log_path + "/examples/aqp_opt.bag", rosbag::bagmode::Write);
	bag.write("cartesian_trajectory", ros::Time::now(), cartTraj);
	bag.write("joint_trajectory", ros::Time::now(), jointTraj);
	bag.close();
	return 0;
}
