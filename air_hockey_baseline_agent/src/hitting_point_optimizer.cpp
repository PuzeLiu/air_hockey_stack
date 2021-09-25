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
#include <iostream>
#include "air_hockey_baseline_agent/hitting_point_optimizer.h"

using namespace std;
using namespace nlopt;
using namespace Eigen;
using namespace air_hockey_baseline_agent;


HittingPointOptimizer::HittingPointOptimizer(AgentParams &agentParameters, OptimizerData &optimizerData) :
		agentParams(agentParameters), optData(optimizerData) {
	nlSolver = opt(LD_SLSQP, agentParams.pinoModel.nq);
	nlSolver.set_max_objective(objective, &optData);

	double tol = 1e-4;
	nlSolver.add_equality_constraint(this->equalityConstraint, &optData, tol);
//	nlSolver.add_inequality_constraint(this->inEqualityConstraint, &optData, 0);

	std::vector<double> qLower(agentParams.pinoModel.lowerPositionLimit.data(),
	                           agentParams.pinoModel.lowerPositionLimit.data() +
	                           agentParams.pinoModel.lowerPositionLimit.cols() *
	                           agentParams.pinoModel.lowerPositionLimit.rows());
	nlSolver.set_lower_bounds(qLower);

	std::vector<double> qUpper(agentParams.pinoModel.upperPositionLimit.data(),
	                           agentParams.pinoModel.upperPositionLimit.data() +
	                           agentParams.pinoModel.upperPositionLimit.cols() *
	                           agentParams.pinoModel.upperPositionLimit.rows());
	nlSolver.set_upper_bounds(qUpper);

	// Set tolerance for nonlinear solver
	nlSolver.set_ftol_abs(1e-6);
	nlSolver.set_xtol_abs(1e-8);

	// Set up linear programming solver
	simplexModel.setLogLevel(0);
}

HittingPointOptimizer::~HittingPointOptimizer() {

}

bool HittingPointOptimizer::solve(const Eigen::Vector3d &hitPoint, const Eigen::Vector3d &hitDirection,
                                  JointArrayType &qInOut, double &velMagMax) {
	optData.hitPoint = hitPoint;
	optData.hitDirection = hitDirection;

	getInitPoint(qInOut);

	std::vector<double> qCur(qInOut.data(), qInOut.data() + qInOut.rows() * qInOut.cols());
	std::vector<double> grad(7);

	double opt_fun;
	auto result = nlSolver.optimize(qCur, opt_fun);

	if (result < 0) {
		velMagMax = 2;
		return false;
	}

	if (h(qCur, &optData) < 1e-4) {
		qInOut = JointArrayType::Map(qCur.data(), qCur.size());

		//! LP optimization
		velMagMax = getMaxVelocityLP(qInOut);
		return true;
	} else {
		cout << "The position error is : " << h(qCur, &optData) << " bigger than 1e-4" << endl;
		velMagMax = 2;
		return false;
	}
}

double HittingPointOptimizer::objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data) {
	auto optData = (OptimizerData *) f_data;
	if (!grad.empty()) {
		numerical_grad(f, x, optData, grad);
	}
	return f(x, optData);
}

double HittingPointOptimizer::equalityConstraint(const vector<double> &x, vector<double> &grad, void *f_data) {
	auto optData = (OptimizerData *) f_data;
	if (!grad.empty()) {
		numerical_grad(h, x, optData, grad);
	}
	return h(x, optData);
}

double HittingPointOptimizer::inEqualityConstraint(const vector<double> &x, vector<double> &grad, void *f_data) {
	auto optData = (OptimizerData *) f_data;
	if (!grad.empty()) {
		numerical_grad(g, x, optData, grad);
	}
	return g(x, optData);
}

double HittingPointOptimizer::f(const vector<double> &x, const OptimizerData *data) {
	Eigen::Map<const JointArrayType> qCur(x.data(), x.size());
	pinocchio::Data::Matrix6x jacobian(6, data->agentParams.pinoModel.nv);
	pinocchio::computeFrameJacobian(data->agentParams.pinoModel, data->agentParams.pinoData, qCur,
	                                data->agentParams.pinoFrameId, pinocchio::LOCAL_WORLD_ALIGNED,
	                                jacobian);
	auto vec = data->hitDirection.transpose() * jacobian.topRows(3);
	return vec.squaredNorm();
}

double HittingPointOptimizer::h(const vector<double> &x, const OptimizerData *data) {
	JointArrayType qCur = JointArrayType::Map(x.data(), x.size());

	Eigen::Vector3d xCur;
	pinocchio::forwardKinematics(data->agentParams.pinoModel, data->agentParams.pinoData, qCur);
	pinocchio::updateFramePlacements(data->agentParams.pinoModel, data->agentParams.pinoData);
	xCur = data->agentParams.pinoData.oMf[data->agentParams.pinoFrameId].translation();

	auto distance = (xCur - data->hitPoint).norm();
	return distance;
}

double HittingPointOptimizer::g(const vector<double> &x, const OptimizerData *data) {
	JointArrayType qCur = JointArrayType::Map(x.data(), x.size());
	pinocchio::forwardKinematics(data->agentParams.pinoModel, data->agentParams.pinoData, qCur);
	pinocchio::updateFramePlacements(data->agentParams.pinoModel, data->agentParams.pinoData);
	int elbow_index = data->agentParams.pinoModel.getFrameId("F_joint_4");
	return 0.2 - data->agentParams.pinoData.oMf[elbow_index].translation()[2];
}

void HittingPointOptimizer::numerical_grad(HittingPointOptimizer::functype function, const vector<double> &x,
                                           const OptimizerData *data, vector<double> &grad) {
	vector<double> x_pos, x_neg;
	for (int i = 0; i < x.size(); ++i) {
		x_pos = x;
		x_neg = x;
		x_pos[i] += data->epsilon;
		x_neg[i] -= data->epsilon;
		grad[i] = (function(x_pos, data) - function(x_neg, data)) / (2 * data->epsilon);
	}
}

bool HittingPointOptimizer::getInitPoint(JointArrayType &qInOut) {
	if (!inverseKinematicsPosition(agentParams, optData.hitPoint, qInOut, qInOut, 1e-4, 200)) {
		qInOut = qInOut.cwiseMax(agentParams.pinoModel.lowerPositionLimit).cwiseMin(
				agentParams.pinoModel.upperPositionLimit);
		return false;
	}
	return true;
}

double HittingPointOptimizer::getMaxVelocity(const JointArrayType &q) {
	pinocchio::Data::Matrix6x jacobian(6, agentParams.pinoModel.nv);
	pinocchio::computeFrameJacobian(agentParams.pinoModel, agentParams.pinoData, q,
	                                agentParams.pinoFrameId, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
	auto jac = jacobian.topRows(3);
	auto qRef = jac.transpose() * (jac * jac.transpose()).ldlt().solve(optData.hitDirection);

	auto min_scale = agentParams.pinoModel.velocityLimit.cwiseProduct(qRef.cwiseAbs().cwiseInverse()).minCoeff();
//	auto q_res = jac.transpose() * (jac * jac.transpose()).ldlt().solve(min_scale * optData.hitDirection);
	return min_scale;
}

double HittingPointOptimizer::getMaxVelocityLP(const JointArrayType &q) {
	pinocchio::Data::Matrix6x jacobian(6, agentParams.pinoModel.nv);
	pinocchio::computeFrameJacobian(agentParams.pinoModel, agentParams.pinoData, q,
	                                agentParams.pinoFrameId, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
	auto jac = jacobian.topRows(3);

	MatrixXd orthogonalComplement;
	getNullSpace(optData.hitDirection.transpose(), orthogonalComplement);

	MatrixXd OCJac = (orthogonalComplement.transpose() * jac);
	MatrixXd OCJacNullSpace;
	getNullSpace(OCJac, OCJacNullSpace);

	// Get Objective
	MatrixXd objective = -optData.hitDirection.transpose() * jac * OCJacNullSpace;

	// Get Bounds for Primal Variables
	int nRows = OCJacNullSpace.rows();
	int nCols = OCJacNullSpace.cols();
	VectorXd columnLower(nCols);
	VectorXd columnUpper(nCols);
	columnLower = -columnLower.setOnes() * COIN_DBL_MAX;
	columnUpper = columnUpper.setOnes() * COIN_DBL_MAX;

	// Get Constraint Matrix
	CoinPackedMatrix matrix;
	matrix.setDimensions(nRows, nCols);
	for (int i = 0; i < OCJacNullSpace.rows(); ++i) {
		for (int j = 0; j < OCJacNullSpace.cols(); ++j) {
			matrix.modifyCoefficient(i, j, OCJacNullSpace(i, j));
		}
	}

	// Generate Problem
	Eigen::VectorXd velLowerLimit = -agentParams.pinoModel.velocityLimit;
	simplexModel.loadProblem(matrix, columnLower.data(), columnUpper.data(), objective.data(),
	                         velLowerLimit.data(), agentParams.pinoModel.velocityLimit.data());
	// Get Solution
	simplexModel.initialSolve();
	simplexModel.primal(0);
	return -simplexModel.getObjValue();
}


