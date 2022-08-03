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

#include "air_hockey_baseline_agent/utils.h"
#include "air_hockey_baseline_agent/null_space_optimizer.h"
#include "osqp/osqp.h"

using namespace std;
using namespace air_hockey_baseline_agent;
using namespace Eigen;

NullSpaceOptimizer::NullSpaceOptimizer(AgentParams &agentParams_, OptimizerData &optimizerData) :
	agentParams(agentParams_), optData(optimizerData) {
	solver.settings()->setWarmStart(true);
	solver.settings()->setVerbosity(false);
	solver.data()->setNumberOfVariables(4);
	solver.data()->setNumberOfConstraints(agentParams.nq);

	if (!solver.data()->setHessianMatrix(optData.P)) { cout << "Set Hessian Error!" << endl; }
	if (!solver.data()->setGradient(optData.q)) { cout << "Set Gradient Error!" << endl; }
	if (!solver.data()->setLinearConstraintsMatrix(optData.A)) { cout << "Set Constraint Matrix Error!" << endl; }

	VectorXd vLimit = -agentParams_.pinoModel.velocityLimit;
	solver.data()->setBounds(vLimit, agentParams_.pinoModel.velocityLimit);

	solver.initSolver();

	simplexModel.setLogLevel(0);
}

NullSpaceOptimizer::~NullSpaceOptimizer() = default;

bool NullSpaceOptimizer::optimizeJointTrajectory(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
												 const JointArrayType &qStart, const JointArrayType &dqStart,
												 trajectory_msgs::JointTrajectory &jointTraj) {
	if (!cartTraj.points.empty()) {
		Vector3d xDes, dxDes;
		JointArrayType qCur, dqCur, qNext, dqNext;
		qCur = qStart;
		dqCur = dqStart;
		optData.anchorAlpha = 0.;

		trajectory_msgs::JointTrajectoryPoint jointViaPoint_;
		jointViaPoint_.positions.resize(agentParams.nq);
		jointViaPoint_.velocities.resize(agentParams.nq);
		for (int i = 0; i < agentParams.nq; ++i) {
			jointViaPoint_.positions[i] = qStart[i];
			jointViaPoint_.velocities[i] = dqStart[i];
		}
		jointViaPoint_.time_from_start = ros::Duration(0.);

		for (size_t i = 0; i < cartTraj.points.size(); ++i) {
			xDes[0] = cartTraj.points[i].transforms[0].translation.x;
			xDes[1] = cartTraj.points[i].transforms[0].translation.y;
			xDes[2] = cartTraj.points[i].transforms[0].translation.z;

			dxDes[0] = cartTraj.points[i].velocities[0].linear.x;
			dxDes[1] = cartTraj.points[i].velocities[0].linear.y;
			dxDes[2] = cartTraj.points[i].velocities[0].linear.z;

			double dt = (cartTraj.points[i].time_from_start - jointViaPoint_.time_from_start).toSec();

			if (!solveQP(xDes, dxDes, qCur, dt, dqNext)) {
				ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Optimization failed at : " << i);
				ROS_DEBUG_STREAM_NAMED(agentParams.name,
									   agentParams.name + ": " + "qNext: " << qNext.transpose() << "dqNext: "
																		   << dqNext.transpose());
				return false;
			}
			qNext = qCur + (dqCur + dqNext) / 2 * dt;
			solveJoint7(qNext, dqNext);

			jointViaPoint_.time_from_start = cartTraj.points[i].time_from_start;
			for (size_t row = 0; row < agentParams.nq; row++) {
				jointViaPoint_.velocities[row] = dqNext[row];
				jointViaPoint_.positions[row] = qNext[row];
			}
			jointTraj.points.push_back(jointViaPoint_);

			qCur = qNext;
			dqCur = dqNext;
		}
		jointTraj.header.stamp = cartTraj.header.stamp;
		return true;
	} else {
		return false;
	}
}

bool NullSpaceOptimizer::optimizeJointTrajectoryAnchor(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
													   const JointArrayType &qStart, const JointArrayType &dqStart,
													   const JointArrayType &qAnchor, double switching_time,
													   trajectory_msgs::JointTrajectory &jointTraj) {
	if (!cartTraj.points.empty()) {
		Vector3d xDes, dxDes;
		JointArrayType qCur, dqCur, qNext, dqNext, dqAnchorTmp;
		double scaleVel;

		qCur = qStart;
		dqCur = dqStart;

		trajectory_msgs::JointTrajectoryPoint jointViaPoint_;
		jointViaPoint_.positions.resize(agentParams.nq);
		jointViaPoint_.velocities.resize(agentParams.nq);
		for (int i = 0; i < agentParams.nq; ++i) {
			jointViaPoint_.positions[i] = qStart[i];
			jointViaPoint_.velocities[i] = dqStart[i];
		}

		jointViaPoint_.time_from_start = ros::Duration(0);

		for (size_t i = 0; i < cartTraj.points.size(); ++i) {
			xDes[0] = cartTraj.points[i].transforms[0].translation.x;
			xDes[1] = cartTraj.points[i].transforms[0].translation.y;
			xDes[2] = cartTraj.points[i].transforms[0].translation.z;

			dxDes[0] = cartTraj.points[i].velocities[0].linear.x;
			dxDes[1] = cartTraj.points[i].velocities[0].linear.y;
			dxDes[2] = cartTraj.points[i].velocities[0].linear.z;

			if (qAnchor.size() != qCur.size()) {
				dqAnchorTmp.setZero(qCur.size());
			} else {
				dqAnchorTmp = (qAnchor - qCur) / 0.5;
			}

			if ((cartTraj.points[i].time_from_start - cartTraj.points[0].time_from_start).toSec() <= switching_time) {
				//! Multiply the phase variable
				optData.anchorAlpha =
					(cartTraj.points[i].time_from_start - cartTraj.points[0].time_from_start).toSec() / switching_time;
				scaleVel = std::min(std::max(1 - optData.anchorAlpha, 0.), 1.);
			} else {
				optData.anchorAlpha =
					(cartTraj.points.back().time_from_start - cartTraj.points[i].time_from_start).toSec() /
						(cartTraj.points.back().time_from_start.toSec() - switching_time);
				scaleVel = 0.;
			}

			dqAnchorTmp = dqAnchorTmp * optData.anchorAlpha + dqStart * scaleVel;

			double dt = (cartTraj.points[i].time_from_start - jointViaPoint_.time_from_start).toSec();

			if (!solveQPAnchor(xDes, dxDes, qCur, dqAnchorTmp, dt, dqNext)) {
				ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "qAchor : " << qAnchor.transpose());
				ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "Optimization failed at : " << i);
				return false;
			}
			qNext = qCur + (dqCur + dqNext) / 2 * dt;
			solveJoint7(qNext, dqNext);

			jointViaPoint_.time_from_start = cartTraj.points[i].time_from_start;
			for (size_t row = 0; row < agentParams.nq; row++) {
				jointViaPoint_.velocities[row] = dqNext[row];
				jointViaPoint_.positions[row] = qNext[row];
			}
			jointTraj.points.push_back(jointViaPoint_);

			qCur = qNext;
			dqCur = dqNext;
		}
		jointTraj.header.stamp = cartTraj.header.stamp;
		optData.anchorPos = qAnchor;
		return true;
	} else {
		return false;
	}
}

bool NullSpaceOptimizer::solveQP(const Vector3d &xDes,
								 const Vector3d &dxDes,
								 const JointArrayType &qCur,
								 const double dt,
								 JointArrayType &dqNext) {
	Eigen::MatrixXd J_tmp(6, agentParams.pinoModel.nv);
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, qCur);
	pinocchio::computeJointJacobians(agentParams.pinoModel, agentParams.pinoData);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	pinocchio::getFrameJacobian(agentParams.pinoModel, agentParams.pinoData,
								agentParams.pinoFrameId, pinocchio::LOCAL_WORLD_ALIGNED, J_tmp);
	optData.xCurPos = agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();
	optData.J = J_tmp.topRows(3);
	getNullSpace(optData.J, optData.N_J);
	optData.K = 1 / dt;

	VectorXd b = optData.J.transpose() * (optData.J * optData.J.transpose()).ldlt().solve(
		(optData.K * (xDes - optData.xCurPos)));

	if (optData.N_J.cols() != optData.dimNullSpace) {
		ROS_ERROR_STREAM("Null space of jacobian should be:" << optData.dimNullSpace);
		return false;
	}

	optData.P = (optData.N_J.transpose() * optData.weights.asDiagonal() * optData.N_J).sparseView();
	optData.q = b.transpose() * optData.weights.asDiagonal() * optData.N_J;
	optData.A = optData.N_J.sparseView();

	optData.upperBound = agentParams.pinoModel.velocityLimit.cwiseMin(
		(agentParams.pinoModel.upperPositionLimit * 0.95 - qCur) * agentParams.rate) - b;
	optData.lowerBound = (-agentParams.pinoModel.velocityLimit).cwiseMax(
		(agentParams.pinoModel.lowerPositionLimit * 0.95 - qCur) * agentParams.rate) - b;

	if (!constructQPSolver()) {
		ROS_DEBUG_STREAM("Unable to construct the QR solver");
		return false;
	}

	VectorXd feasiblePoint;
	if (not checkFeasibility(optData.A.toDense(), optData.lowerBound, optData.upperBound, feasiblePoint)) {
		ROS_DEBUG_STREAM("Infeasible Constraints!");
		return false;
	}
	solver.setPrimalVariable(feasiblePoint);

	if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError
		and solver.getStatus() == OsqpEigen::Status::Solved) {
		optData.alphaLast = solver.getSolution();
		dqNext = b + optData.N_J * solver.getSolution();
		return true;
	}
	return false;
}

bool NullSpaceOptimizer::solveQPAnchor(const Vector3d &xDes,
									   const Vector3d &dxDes,
									   const JointArrayType &qCur,
									   const JointArrayType &dqAnchor,
									   const double dt,
									   JointArrayType &dqNext) {
	Eigen::MatrixXd J_tmp(6, agentParams.pinoModel.nv);
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, qCur);
	pinocchio::computeJointJacobians(agentParams.pinoModel, agentParams.pinoData);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
	pinocchio::getFrameJacobian(agentParams.pinoModel, agentParams.pinoData,
								agentParams.pinoFrameId, pinocchio::LOCAL_WORLD_ALIGNED, J_tmp);
	optData.xCurPos = agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();
	optData.J = J_tmp.topRows(3);
	getNullSpace(optData.J, optData.N_J);
	optData.K = 1 / dt;

	VectorXd b = optData.J.transpose() * (optData.J * optData.J.transpose()).ldlt().solve(
		optData.K * (xDes - optData.xCurPos));
	MatrixXd I(7, 7);
	I.setIdentity();
	VectorXd omega = b - dqAnchor;

	if (optData.N_J.cols() != optData.dimNullSpace) {
		ROS_ERROR_STREAM("Null space of jacobian should be:" << optData.dimNullSpace);
		return false;
	}

	optData.P = (optData.N_J.transpose() * optData.weightsAnchor.asDiagonal() * optData.N_J).sparseView();

	optData.q = omega.transpose() * optData.weightsAnchor.asDiagonal() * optData.N_J;
	optData.A = optData.N_J.sparseView();

	optData.upperBound = (agentParams.pinoModel.velocityLimit * agentParams.hitVelocityScale).cwiseMin(
		(agentParams.pinoModel.upperPositionLimit * 0.95 - qCur) * agentParams.rate) - b;
	optData.lowerBound = (-agentParams.pinoModel.velocityLimit * agentParams.hitVelocityScale).cwiseMax(
		(agentParams.pinoModel.lowerPositionLimit * 0.95 - qCur) * agentParams.rate) - b;

	if (!constructQPSolver()) {
		ROS_DEBUG_STREAM("Unable to construct the QR solver");
		return false;
	}

	VectorXd feasiblePoint;
	if (not checkFeasibility(optData.A.toDense(), optData.lowerBound, optData.upperBound, feasiblePoint)) {
		ROS_DEBUG_STREAM("Infeasible Constraints!");
		return false;
	}
	solver.setPrimalVariable(feasiblePoint);

	if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError
		and solver.getStatus() == OsqpEigen::Status::Solved) {
		optData.alphaLast = solver.getSolution();
		dqNext = b + optData.N_J * solver.getSolution();
		return true;
	}
	return false;
}

void NullSpaceOptimizer::solveJoint7(JointArrayType &q, JointArrayType &dq) {
	double qCur7 = q[6];
	// Set last joint to 0 solve desired joint position
	q[6] = 0.0;

	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, q);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);

	Matrix3d mat = agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation();
	Vector3d zAxis(0., 0., -1.);
	auto yDes = zAxis.cross(mat.col(2)).normalized();
	double target = acos(boost::algorithm::clamp(mat.col(1).dot(yDes), -1., 1.));
	Vector3d axis = mat.col(1).cross(yDes).normalized();
	target = target * axis.dot(mat.col(2));

	if (target - qCur7 > M_PI_2) {
		target -= M_PI;
	} else if (target - qCur7 < -M_PI_2) {
		target += M_PI;
	}

	dq[6] = boost::algorithm::clamp((target - qCur7) * agentParams.rate,
									-agentParams.pinoModel.velocityLimit[6],
									agentParams.pinoModel.velocityLimit[6]);
	q[6] = boost::algorithm::clamp(qCur7 + dq[6] / agentParams.rate,
								   agentParams.pinoModel.lowerPositionLimit[6],
								   agentParams.pinoModel.upperPositionLimit[6]);
}

bool NullSpaceOptimizer::constructQPSolver(bool verbose) {
	solver.clearSolver();
	solver.data()->clearHessianMatrix();
	solver.data()->clearLinearConstraintsMatrix();

	solver.settings()->setVerbosity(verbose);
	solver.data()->setNumberOfVariables(optData.dimNullSpace);
	solver.data()->setNumberOfConstraints(agentParams.nq);

	if (!solver.data()->setHessianMatrix(optData.P)) {
		ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "setHessianMatrix Failed.");
		return false;
	}
	if (!solver.data()->setGradient(optData.q)) {
		ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "setGradient Failed.");
		return false;
	}
	if (!solver.data()->setLinearConstraintsMatrix(optData.A)) {
		ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "setLinearConstraintsMatrix Failed.");
		return false;
	}
	if ((optData.lowerBound.array() > optData.upperBound.array()).any() or
		!solver.data()->setBounds(optData.lowerBound, optData.upperBound)) {
		ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "setBounds Failed.");
		return false;
	}
	solver.settings()->setWarmStart(true);
	if (!solver.initSolver()) {
		ROS_DEBUG_STREAM_NAMED(agentParams.name, agentParams.name + ": " + "initSolver Failed.");
		return false;
	}
	return true;
}

bool NullSpaceOptimizer::checkFeasibility(const MatrixXd A,
										  const VectorXd lowerBound,
										  const VectorXd upperBound,
										  VectorXd &feasiblePoint) {
	// Get Constraint Matrix
	CoinPackedMatrix matrix;
	matrix.setDimensions(A.rows(), A.cols());
	for (int i = 0; i < A.rows(); ++i) {
		for (int j = 0; j < A.cols(); ++j) {
			matrix.modifyCoefficient(i, j, A(i, j));
		}
	}

	VectorXd columnLower(A.cols());
	VectorXd columnUpper(A.cols());
	columnLower = -columnLower.setOnes() * COIN_DBL_MAX;
	columnUpper = columnUpper.setOnes() * COIN_DBL_MAX;

	MatrixXd obj(A.cols(), A.cols());
	obj.setIdentity();

	simplexModel.loadProblem(matrix, columnLower.data(), columnUpper.data(), obj.data(),
							 lowerBound.data(), upperBound.data());
	simplexModel.initialSolve();
	simplexModel.primal();

	feasiblePoint = VectorXd::Map(simplexModel.primalColumnSolution(), simplexModel.getNumCols());
	feasiblePoint(feasiblePoint.rows() - 1) = 0.; // Set value of joint 7 to zero
	return simplexModel.primalFeasible();
}

