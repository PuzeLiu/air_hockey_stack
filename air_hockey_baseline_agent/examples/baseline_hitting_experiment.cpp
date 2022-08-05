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
	plannerStatusPub = nh.advertise<air_hockey_msgs::PlannerStatus>("/neural_planner/status", 1);
	get_hitting_state_client = nh.serviceClient<air_hockey_msgs::GetHittingState>("/iiwa_front/get_hitting_state");
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
	ros::Duration(1.0).sleep();

	ROS_INFO_STREAM("GoTo Initial Configuration");
	trajectory_msgs::JointTrajectoryPoint initPoint;
	initPoint.positions = std::vector<double>(agent.getAgentParams().qHome.data(),
											  agent.getAgentParams().qHome.data() + agent.getAgentParams().qHome.size());
	initPoint.velocities.resize(agent.getAgentParams().nq, 0.);
	initPoint.time_from_start = ros::Duration(3.0);
	agent.getState().trajectoryBuffer.getFree().jointTrajectory.points.clear();
	agent.getState().trajectoryBuffer.getFree().jointTrajectory.points.push_back(initPoint);
	agent.getState().trajectoryBuffer.getFree().jointTrajectory.header.stamp = ros::Time::now();

	trajectory_msgs::MultiDOFJointTrajectoryPoint initCartPoint;
	geometry_msgs::Transform transform_tmp;
	transform_tmp.translation.x = agent.getAgentParams().xHome[0];
	transform_tmp.translation.y = agent.getAgentParams().xHome[1];
	transform_tmp.translation.z = agent.getAgentParams().xHome[2];
	initCartPoint.time_from_start = initPoint.time_from_start;
	initCartPoint.transforms.push_back(transform_tmp);
	agent.getState().trajectoryBuffer.getFree().cartTrajectory.points.clear();
	agent.getState().trajectoryBuffer.getFree().cartTrajectory.points.push_back(initCartPoint);
	agent.getState().trajectoryBuffer.getFree().cartTrajectory.header.stamp =
		agent.getState().trajectoryBuffer.getFree().jointTrajectory.header.stamp;

	agent.publishTrajectory();
	ros::Duration(4.0).sleep();
	ROS_INFO_STREAM("Wait for the topic");
}

void BaselineHittingExperiment::start() {
	ros::Rate rate(agent.getAgentParams().rate);
	while (ros::ok()) {
		if (publishStatus) {
			if (planHittingTrajectory()) {
				agent.getState().trajectoryBuffer.getFree().jointTrajectory = jointTraj;
				agent.getState().trajectoryBuffer.getFree().cartTrajectory = cartTraj;
				agent.publishTrajectory();
                (cartTraj.points.back().time_from_start).sleep();
                planGettingBackTrajectory();
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
	// Hitting point optimization by Puze
	//agent.getTrajectoryGenerator().hittingPointOptimizer->solve(hitPos, hitDir, qHitRef, hitVelMag);
	// Hitting point optimization by IK NN
	air_hockey_msgs::GetHittingState srv;
	srv.request.x = hitPos[0];
	srv.request.y = hitPos[1];
    //std::cout << "HIT POS: " <<  hitPos[0] << "  " << hitPos[1] <<  std::endl;
	srv.request.th = atan2(hitDir[1], hitDir[0]);
	get_hitting_state_client.call(srv);
    //std::cout << "CHECKING IF VELOCITIES ARE THERE:" << std::endl;
	if (srv.response.magnitude == 0.) {return false;}
    //std::cout << "HITTING VELOCITY FROM MY OPT:" << std::endl;
	for (auto i = 0; i < 6; ++i){
	    qHitRef[i] = srv.response.q[i];
        //std::cout << srv.response.q_dot[i] << " ";
	}
    //std::cout << std::endl;
	hitVelMag = srv.response.magnitude;

	agent.getTrajectoryGenerator().transformations->transformRobot2Table(hitPos);
	agent.getTrajectoryGenerator().transformations->rotationRobot2Table(hitDir);

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
	double tStop = (xEnd - agent.getAgentParams().xHome).norm() / 0.7;
	ROS_INFO_STREAM(xStart.transpose() << " v: " << vStart.transpose() << "vHitMag: " << hitVelMag << " end: " << xEnd.transpose());

	for (int i = 0; i < 10; ++i) {
		cartTraj.points.clear();
		jointTraj.points.clear();

		hitVel = hitDir * hitVelMag;
        ROS_INFO_STREAM("HITVEL: " << hitVel);

		agent.getTrajectoryGenerator().combinatorialHitNew->plan(xStart, vStart, hitPos, hitVel, hittingTime,
		                                                         xEnd, Eigen::Vector3d::Zero(), cartTraj);
//		agent.getTrajectoryGenerator().cubicLinearMotion->plan(xEnd, Eigen::Vector3d::Zero(),
//															   agent.getAgentParams().xHome, Eigen::Vector3d::Zero(),
//															   tStop, cartTraj);
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
            ROS_INFO_STREAM("Scaled down");
			hitVelMag *= 0.9;
		}
	}
	ROS_INFO_STREAM("Optimization Hitting Failed");
	plannerStatusMsg.planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count();
	plannerStatusMsg.header.stamp = jointTraj.header.stamp;
	plannerStatusMsg.success = false;
	plannerStatusMsg.planned_hitting_time = -1.;
	plannerStatusMsg.planned_motion_time = -1.;
	plannerStatusMsg.planned_hit_joint_velocity.clear();
	plannerStatusMsg.planned_hit_cartesian_velocity.clear();
	return false;
}

bool BaselineHittingExperiment::planGettingBackTrajectory() {
	JointArrayType qStartTmp = JointArrayType::Map(jointTraj.points.back().positions.data(), jointTraj.points.back().positions.size());
	JointArrayType dqStartTmp = JointArrayType::Map(jointTraj.points.back().velocities.data(), jointTraj.points.back().velocities.size());
    Eigen::Vector3d xEnd(cartTraj.points.back().transforms[0].translation.x,
                         cartTraj.points.back().transforms[0].translation.y,
                         cartTraj.points.back().transforms[0].translation.z);
    agent.getTrajectoryGenerator().transformations->transformRobot2Table(xEnd);

	double tStop = 2.0;
	for (int j = 0; j < 10; ++j) {
		cartTraj.points.clear();
		jointTraj.points.clear();
		agent.getTrajectoryGenerator().cubicLinearMotion->plan(xEnd, Eigen::Vector3d::Zero(),
															   agent.getAgentParams().xHome, Eigen::Vector3d::Zero(), tStop, cartTraj);
		agent.getTrajectoryGenerator().transformations->transformTrajectory(cartTraj);

		prePlanPoint.time_from_start = ros::Duration(0.);
		prePlanPoint.positions = std::vector<double>(qStartTmp.data(), qStartTmp.data() + qStartTmp.size());
		prePlanPoint.velocities = std::vector<double>(dqStartTmp.data(), dqStartTmp.data() + dqStartTmp.size());
		if(agent.getTrajectoryGenerator().optimizer->optimizeJointTrajectoryAnchor(cartTraj, qStartTmp, dqStartTmp, agent.getAgentParams().qHome, tStop, jointTraj)){
			agent.getTrajectoryGenerator().cubicSplineInterpolation(jointTraj, prePlanPoint);
			agent.getTrajectoryGenerator().synchronizeCartesianTrajectory(jointTraj,cartTraj);
			jointTraj.header.stamp = ros::Time::now() + ros::Duration(0.1);
			cartTraj.header.stamp = jointTraj.header.stamp;

			agent.getState().trajectoryBuffer.getFree().jointTrajectory = jointTraj;
			agent.getState().trajectoryBuffer.getFree().cartTrajectory = cartTraj;
			agent.publishTrajectory();
			return true;
		} else {
			tStop *= 1.2;
		}
	}
	ROS_INFO_STREAM("Optimization Move Back Failed");
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
