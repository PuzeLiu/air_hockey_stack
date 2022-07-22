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

#include "air_hockey_baseline_agent/trajectory_generator.h"

using namespace Eigen;
using namespace air_hockey_baseline_agent;

TrajectoryGenerator::TrajectoryGenerator(const ros::NodeHandle &nh, AgentParams &agentParams,
										 EnvironmentParams &envParams) : agentParams(agentParams),
																		 envParams(envParams),
																		 optData(agentParams) {
	initOptimizerData(nh);
	optimizer = new NullSpaceOptimizer(agentParams, optData);
	transformations = new Transformations(nh.getNamespace());
	hittingPointOptimizer = new HittingPointOptimizer(agentParams, optData);

	Vector2d bound_lower(envParams.malletRadius + 0.02,
						 -envParams.tableWidth / 2 + envParams.malletRadius + 0.02);
	Vector2d bound_upper(envParams.tableLength / 2 - envParams.malletRadius,
						 envParams.tableWidth / 2 - envParams.malletRadius - 0.02);

	combinatorialHitNew = new CombinatorialHitNew(bound_lower, bound_upper, agentParams.rate,
												  agentParams.universalJointHeight, agentParams.eeMaxAcceleration);
	cubicLinearMotion = new CubicLinearMotion(agentParams.rate, agentParams.universalJointHeight);
}

void TrajectoryGenerator::getCartesianPosAndVel(Vector3d &x, Vector3d &dx,
												JointArrayType &q, JointArrayType &dq) {
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, q, dq);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	x = agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();
	dx = pinocchio::getFrameVelocity(agentParams.pinoModel, agentParams.pinoData,
									 agentParams.pinoFrameId, pinocchio::LOCAL_WORLD_ALIGNED).linear();
}

TrajectoryGenerator::~TrajectoryGenerator() {
	delete optimizer;
	delete transformations;
	delete hittingPointOptimizer;
	delete combinatorialHitNew;
	delete cubicLinearMotion;
}

void TrajectoryGenerator::cubicSplineInterpolation(trajectory_msgs::JointTrajectory &jointTraj,
												   trajectory_msgs::JointTrajectoryPoint &planPrevPoint) {
	//! Add two point before planning to smooth the connection
	jointTraj.points.insert(jointTraj.points.begin(), planPrevPoint);
	int n = jointTraj.points.size();
	std::vector<double> x(n);
	std::vector<double> y(n);

	for (int i = 0; i < agentParams.pinoModel.nq; ++i) {
		for (int j = 0; j < n; ++j) {
			x[j] = jointTraj.points[j].time_from_start.toSec();
			y[j] = jointTraj.points[j].positions[i];
			jointTraj.points[j].velocities.resize(agentParams.pinoModel.nq);
			jointTraj.points[j].accelerations.clear();
		}
		tk::spline spline(x, y, tk::spline::cspline, false,
						  tk::spline::first_deriv, jointTraj.points.front().velocities[i],
						  tk::spline::first_deriv, jointTraj.points.back().velocities[i]);

		for (int j = 0; j < n; ++j) {
			jointTraj.points[j].velocities[i] = spline.deriv(1, x[j]);
		}
	}

//	! Remove the assistant point
	jointTraj.points.erase(jointTraj.points.begin());
}

void TrajectoryGenerator::synchronizeCartesianTrajectory(const trajectory_msgs::JointTrajectory &jointTraj,
														 trajectory_msgs::MultiDOFJointTrajectory &cartTraj) {
	if (jointTraj.points.size() != cartTraj.points.size()) {
		ROS_WARN_STREAM("Joint Trajectory and Cartesian Trajectory does not have the same size");
		cartTraj.points.resize(jointTraj.points.size());
	}
	cartTraj.header.stamp = jointTraj.header.stamp;
	Eigen::Vector3d xTmp, vTmp;
	JointArrayType q, dq;
	trajectory_msgs::MultiDOFJointTrajectoryPoint cartPoint;
	cartPoint.transforms.resize(1);
	cartPoint.velocities.resize(1);

	for (int j = 0; j < jointTraj.points.size(); ++j) {
		q = VectorXd::Map(jointTraj.points[j].positions.data(), jointTraj.points[j].positions.size());
		dq = VectorXd::Map(jointTraj.points[j].velocities.data(), jointTraj.points[j].velocities.size());
		getCartesianPosAndVel(xTmp, vTmp, q, dq);
		cartPoint.transforms[0].translation.x = xTmp.x();
		cartPoint.transforms[0].translation.y = xTmp.y();
		cartPoint.transforms[0].translation.z = xTmp.z();
		cartPoint.velocities[0].linear.x = vTmp.x();
		cartPoint.velocities[0].linear.y = vTmp.y();
		cartPoint.velocities[0].linear.z = vTmp.z();
		cartPoint.time_from_start = jointTraj.points[j].time_from_start;
		cartTraj.points[j] = cartPoint;
	}
}

void TrajectoryGenerator::initOptimizerData(const ros::NodeHandle &nh) {
	optData.K = agentParams.rate;
	optData.weights << 100., 100., 5., 50., 1., 1., 0.01;
	optData.weightsAnchor << 1., 1., 5., 1, 10., 10., 0.01;
}

void TrajectoryGenerator::getPlannedJointState(SystemState &state, ros::Time &tPlan) {
	int idx = -1;

	//! Check if tPlan is in the executing trajectory
	if (tPlan >= state.trajectoryBuffer.getExec().jointTrajectory.header.stamp) {
		idx = state.trajectoryBuffer.getExec().jointTrajectory.points.size() - 1;
		for (int j = 0; j < state.trajectoryBuffer.getExec().jointTrajectory.points.size(); ++j) {
			if (tPlan <= state.trajectoryBuffer.getExec().jointTrajectory.header.stamp +
				state.trajectoryBuffer.getExec().jointTrajectory.points[j].time_from_start) {
				tPlan = state.trajectoryBuffer.getExec().jointTrajectory.header.stamp +
					state.trajectoryBuffer.getExec().jointTrajectory.points[j].time_from_start;
				idx = j;
				break;
			}
		}

		state.qPlan = JointArrayType::Map(
			state.trajectoryBuffer.getExec().jointTrajectory.points[idx].positions.data(), agentParams.nq);
		state.dqPlan = JointArrayType::Map(
			state.trajectoryBuffer.getExec().jointTrajectory.points[idx].velocities.data(), agentParams.nq);
		state.planPrevPoint = state.trajectoryBuffer.getExec().jointTrajectory.points[idx];
		state.planPrevPoint.time_from_start = ros::Duration(0);
	}

	//! Check if tPlan is in the future trajectory
	if (tPlan >= state.trajectoryBuffer.getFree().jointTrajectory.header.stamp and
		state.trajectoryBuffer.getFree().jointTrajectory.header.stamp >
			state.trajectoryBuffer.getExec().jointTrajectory.header.stamp) {
		idx = int(state.trajectoryBuffer.getFree().jointTrajectory.points.size()) - 1;
		for (int j = 0; j < state.trajectoryBuffer.getFree().jointTrajectory.points.size(); ++j) {
			if (tPlan <= state.trajectoryBuffer.getFree().jointTrajectory.header.stamp +
				state.trajectoryBuffer.getFree().jointTrajectory.points[j].time_from_start) {
				tPlan = state.trajectoryBuffer.getFree().jointTrajectory.header.stamp +
					state.trajectoryBuffer.getFree().jointTrajectory.points[j].time_from_start;
				idx = j;
				break;
			}
		}

		state.qPlan = JointArrayType::Map(
			state.trajectoryBuffer.getFree().jointTrajectory.points[idx].positions.data(), agentParams.nq);
		state.dqPlan = JointArrayType::Map(
			state.trajectoryBuffer.getFree().jointTrajectory.points[idx].velocities.data(), agentParams.nq);
		state.planPrevPoint = state.trajectoryBuffer.getFree().jointTrajectory.points[idx];
		state.planPrevPoint.time_from_start = ros::Duration(0);
	}
	//! No trajectory
	if (idx < 0) {
		ROS_ERROR_STREAM("No buffer");
		state.qPlan = state.observation.jointDesiredPosition;
		state.dqPlan = state.observation.jointDesiredVelocity;
		for (int j = 0; j < agentParams.nq; ++j) {
			state.planPrevPoint.positions[j] = state.qPlan[j];
			state.planPrevPoint.velocities[j] = 0.;
		}
		state.planPrevPoint.time_from_start = ros::Duration(0.0);
	}

	getCartesianPosAndVel(state.xPlan, state.vPlan, state.qPlan, state.dqPlan);
	transformations->transformRobot2Table(state.xPlan);
	transformations->rotationRobot2Table(state.vPlan);
}

