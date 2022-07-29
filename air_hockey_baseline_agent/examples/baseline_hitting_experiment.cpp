/*
 * MIT License
 * Copyright (c) 2020-2022 Puze Liu, Davide Tateo, Piotr Kicki
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

#include "examples/baseline_hitting_experiment.h"
#include <chrono>

using namespace air_hockey_baseline_agent;
using namespace baseline_hitting_experiment;

BaselineHittingExperiment::BaselineHittingExperiment(ros::NodeHandle nh) : nh(nh), agent(nh) {
	plannerRequestSub = nh.subscribe("/neural_planner/plan_trajectory", 1, &BaselineHittingExperiment::planRequestCB, this);
	plannerStatusPub = nh.advertise<air_hockey_msgs::PlannerStatus>("plan_status", 1);
	publishStatus = false;

	if (agent.getAgentParams().name == "iiwa_front") {
		cartTraj.header.frame_id = "TableHome";
	} else if (agent.getAgentParams().name == "iiwa_back") {
		cartTraj.header.frame_id = "TableAway";
	}

	cartTraj.joint_names.push_back("x");
	cartTraj.joint_names.push_back("y");
	cartTraj.joint_names.push_back("z");

	for (int j = 0; j < agent.getAgentParams().nq; ++j) {
		jointTraj.joint_names.push_back(agent.getAgentParams().pinoModel.names[j + 1]);
	}
}

void BaselineHittingExperiment::start() {
	ros::Rate rate(agent.getAgentParams().rate);
	while (ros::ok()) {
		if (publishStatus) {
			if (planHittingTrajectory()) {
				jointTrajectoryCmdPub.publish(jointTraj);
				cartTrajectoryCmdPub.publish(cartTraj);
			}
			plannerStatusPub.publish(plannerStatusMsg);
			publishStatus = false;
		}
		ros::spinOnce();
		rate.sleep();
	}
}

bool BaselineHittingExperiment::planHittingTrajectory() {
	auto t_start = std::chrono::high_resolution_clock::now();

	Eigen::Vector3d xStart, vStart;
	double hittingTime, hitVelMag;
	JointArrayType qHitRef;

	agent.getTrajectoryGenerator().getCartesianPosAndVel(xStart, vStart, qStart, dqStart);
	agent.getTrajectoryGenerator().transformations->transformRobot2Table(xStart);
	agent.getTrajectoryGenerator().transformations->rotationRobot2Table(vStart);
	qHitRef = qStart;
	agent.getTrajectoryGenerator().hittingPointOptimizer->solve(hitPos, hitDir, qHitRef, hitVelMag);

	Eigen::Vector3d hitVel, xEnd;
	if (hitPos.y() > 0) {
		xEnd = hitPos + Eigen::Vector3d(0.1, -0.15, 0.0);
	} else {
		xEnd = hitPos + Eigen::Vector3d(0.1, 0.15, 0.0);
	}
	xEnd.x() = fmin(xEnd.x(), agent.getAgentParams().hitRange[1]);
	xEnd.y() = fmax(fmin(xEnd.y(),
	                     agent.getEnvironmentParams().tableWidth / 2 - agent.getEnvironmentParams().malletRadius
		                     - 0.05),
	                -agent.getEnvironmentParams().tableWidth / 2 + agent.getEnvironmentParams().malletRadius + 0.05);
	double tStop = (xEnd - xStart).norm() / 1.0;

	for (int i = 0; i < 10; ++i) {
		cartTraj.points.clear();
		jointTraj.points.clear();

		hitVel = hitDir * hitVelMag;

		agent.getTrajectoryGenerator().combinatorialHitNew->plan(xStart, vStart, hitPos, hitVel, hittingTime,
		                                                         xEnd, Eigen::Vector3d::Zero(), cartTraj);
		agent.getTrajectoryGenerator().cubicLinearMotion->plan(xEnd, Eigen::Vector3d::Zero(),
															   endPoint, Eigen::Vector3d::Zero(),
															   tStop, cartTraj);
		agent.getTrajectoryGenerator().transformations->transformTrajectory(cartTraj);

		if (agent.getTrajectoryGenerator().optimizer
			->optimizeJointTrajectoryAnchor(cartTraj, qStart, dqStart, qHitRef, hittingTime, jointTraj)) {

			agent.getTrajectoryGenerator().cubicSplineInterpolation(jointTraj, prePlanPoint);
			agent.getTrajectoryGenerator().synchronizeCartesianTrajectory(jointTraj,cartTraj);

			jointTraj.header.stamp = ros::Time::now() + ros::Duration(0.1);
			cartTraj.header.stamp = jointTraj.header.stamp;

			plannerStatusMsg.planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count();
			plannerStatusMsg.header.stamp = jointTraj.header.stamp;
			plannerStatusMsg.success = true;
			plannerStatusMsg.planned_hitting_time = hittingTime;
			plannerStatusMsg.planned_motion_time = jointTraj.points.back().time_from_start.toSec();
			int hittingIdx = std::round(hittingTime * agent.getAgentParams().rate);
			plannerStatusMsg.planned_hit_joint_velocity = std::vector<float>(jointTraj.points[hittingIdx].velocities.begin(),
																			 jointTraj.points[hittingIdx].velocities.end());
			plannerStatusMsg.planned_hit_cartesian_velocity.clear();
			plannerStatusMsg.planned_hit_cartesian_velocity.push_back(cartTraj.points[hittingIdx].velocities[0].linear.x);
			plannerStatusMsg.planned_hit_cartesian_velocity.push_back(cartTraj.points[hittingIdx].velocities[0].linear.y);
			plannerStatusMsg.planned_hit_cartesian_velocity.push_back(cartTraj.points[hittingIdx].velocities[0].linear.z);
			return true;
		}
		else {
			hitVelMag *= 0.9;
		}
	}

	plannerStatusMsg.planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count();
	plannerStatusMsg.header.stamp = jointTraj.header.stamp;
	plannerStatusMsg.success = false;
	plannerStatusMsg.planned_hitting_time = -1.;
	plannerStatusMsg.planned_motion_time = -1.;
	plannerStatusMsg.planned_hit_joint_velocity.clear();
	plannerStatusMsg.planned_hit_cartesian_velocity.clear();
	return false;
}

void BaselineHittingExperiment::planRequestCB(air_hockey_msgs::PlannerRequestConstPtr msg) {
	publishStatus = true;
	qStart = Eigen::VectorXf::Map(msg->q_0.data(), msg->q_0.size()).cast<double>();
	dqStart = Eigen::VectorXf::Map(msg->q_dot_0.data(), msg->q_dot_0.size()).cast<double>();
	ddqStart = Eigen::VectorXf::Map(msg->q_ddot_0.data(), msg->q_ddot_0.size()).cast<double>();
	qEnd = Eigen::VectorXf::Map(msg->q_end.data(), msg->q_end.size()).cast<double>();
	hitPos = Eigen::Vector3f(msg->hit_point.x, msg->hit_point.y, msg->hit_point.z).cast<double>();
	hitDir = Eigen::Vector3f(cos(msg->hit_angle), sin(msg->hit_angle), 0.).cast<double>();
	endPoint = Eigen::Vector3f(msg->end_point.x, msg->end_point.y, msg->end_point.z).cast<double>();

	prePlanPoint.time_from_start = ros::Duration(0.);
	prePlanPoint.positions = std::vector<double>(std::begin(msg->q_0), std::end(msg->q_0));
	prePlanPoint.velocities = std::vector<double>(std::begin(msg->q_dot_0), std::end(msg->q_dot_0));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "baseline_hitting_experiment");
	ros::NodeHandle nh("/");

	BaselineHittingExperiment exp(nh);

	exp.start();
	return 0;
}
