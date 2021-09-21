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

	combinatorialHit = new CombinatorialHit(bound_lower, bound_upper, optData.rate,
	                                        envParams.universalJointHeight);
	combinatorialHitNew = new CombinatorialHitNew(bound_lower, bound_upper, optData.rate,
	                                              envParams.universalJointHeight);
	cubicLinearMotion = new CubicLinearMotion(optData.rate, envParams.universalJointHeight);
}

void TrajectoryGenerator::getCartesianPosAndVel(Vector3d &x, Vector3d &dx,
                                                JointArrayType &q, JointArrayType &dq) {
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, q, dq);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	x = agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();
	dx = pinocchio::getFrameVelocity(agentParams.pinoModel, agentParams.pinoData,
	                                 agentParams.pinoFrameId, pinocchio::LOCAL_WORLD_ALIGNED).linear();
//	kinematics->forwardKinematics(q, x);
//	Kinematics::JacobianPosType jacobian;
//	kinematics->jacobianPos(q, jacobian);
//	dx = jacobian * dq;
}

TrajectoryGenerator::~TrajectoryGenerator() {
	delete optimizer;
	delete transformations;
	delete combinatorialHit;
	delete cubicLinearMotion;
}

void TrajectoryGenerator::interpolateAcceleration(trajectory_msgs::JointTrajectory &jointTraj) {
	for (int i = 1; i < jointTraj.points.size() - 1; ++i) {
		auto dt = jointTraj.points[i + 1].time_from_start - jointTraj.points[i - 1].time_from_start;
		jointTraj.points[i].accelerations.resize(7);
		for (int j = 0; j < agentParams.pinoModel.nq; ++j) {
			auto d1 = jointTraj.points[i].velocities[j] - jointTraj.points[i - 1].velocities[j];
			auto d2 = jointTraj.points[i + 1].velocities[j] - jointTraj.points[i].velocities[j];
			if (d1 * d2 < 0) {
				jointTraj.points[i].accelerations[j] = 0;
			} else {
				jointTraj.points[i].accelerations[j] = (d1 + d2) / dt.toSec();
			}
		}
	}
}

void TrajectoryGenerator::interpolateVelocity(trajectory_msgs::JointTrajectory &jointTraj) {
	for (int i = 1; i < jointTraj.points.size() - 1; ++i) {
		auto dt = jointTraj.points[i + 1].time_from_start - jointTraj.points[i - 1].time_from_start;
		for (int j = 0; j < agentParams.pinoModel.nq; ++j) {
			auto d1 = jointTraj.points[i].positions[j] - jointTraj.points[i - 1].positions[j];
			auto d2 = jointTraj.points[i + 1].positions[j] - jointTraj.points[i].positions[j];
			if (d1 * d2 < 0) {
				jointTraj.points[i].velocities[j] = 0;
			} else {
				jointTraj.points[i].velocities[j] = (d1 + d2) / dt.toSec();
			}
		}
	}
}

void TrajectoryGenerator::cubicSplineInterpolation(trajectory_msgs::JointTrajectory &jointTraj) {
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
		                  tk::spline::first_deriv, 0.0,
		                  tk::spline::first_deriv, 0.0);

		for (int j = 0; j < n; ++j) {
			jointTraj.points[j].velocities[i] = spline.deriv(1, x[j]);
		}
	}
}

void TrajectoryGenerator::initOptimizerData(const ros::NodeHandle &nh) {
	nh.getParam("/air_hockey/agent/rate", optData.rate);
	optData.K.setConstant(optData.rate);
	optData.weights << 10., 10., 5., 10., 1., 1., 0.;
	optData.weightsAnchor << 1., 1., 5., 1, 10., 10., 0.;
}

