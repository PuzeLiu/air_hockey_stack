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

#include "air_hockey_baseline_agent/utils.h"

using namespace Eigen;

void air_hockey_baseline_agent::getNullSpace(const Eigen::MatrixXd &A, MatrixXd &out_null_space) {
	CompleteOrthogonalDecomposition<Matrix<double, Dynamic, Dynamic> > cod;
	cod.compute(A);
	// Find URV^T
	MatrixXd V_ = cod.matrixZ().transpose();
	out_null_space = V_.block(0, cod.rank(), V_.rows(), V_.cols() - cod.rank());
//    MatrixXd P = cod.colsPermutation();
	out_null_space = cod.colsPermutation() * out_null_space; // Unpermute the columns
}

bool air_hockey_baseline_agent::inverseKinematics(AgentParams &agentParams, pinocchio::SE3 &oMdes,
                                                  const JointArrayType &q_in, JointArrayType &q_out,
												  double tol, int max_iter) {

	q_out = q_in;

	bool success=false;
	Eigen::VectorXd v(agentParams.pinoModel.nv);
	pinocchio::Data::Matrix6x jac(6, agentParams.pinoModel.nv);

	for (int j = 0; j < max_iter; ++j) {
		pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, q_out);
		pinocchio::computeJointJacobians(agentParams.pinoModel, agentParams.pinoData);
		pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);

		pinocchio::SE3 dMi = agentParams.pinoData.oMf[agentParams.pinoFrameId].actInv(oMdes);
		Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dMi);

		if (err.norm() < tol) {
			if ((agentParams.pinoModel.upperPositionLimit.array() < q_out.array()).any() or
			    (agentParams.pinoModel.lowerPositionLimit.array() > q_out.array()).any()) {
				success = false;
			} else success = true;
			break;
		}
		pinocchio::getFrameJacobian(agentParams.pinoModel, agentParams.pinoData, agentParams.pinoFrameId,
									pinocchio::LOCAL, jac);
		pinocchio::Data::Matrix6 JJt;
		JJt.noalias() = jac * jac.transpose();
		JJt.diagonal().array() += 1e-6;  // Add Damping in Diagonal
		v.noalias() = jac.transpose() * JJt.ldlt().solve(err);
		q_out = q_out + v * 1e-1;
		pinocchio::normalize(agentParams.pinoModel, q_out);

	}
	return success;
}

bool air_hockey_baseline_agent::inverseKinematicsPosition(AgentParams &agentParams, Eigen::Vector3d posDes,
                                                          const JointArrayType &q_in, JointArrayType &q_out,
														  double tol, int max_iter) {
	q_out = q_in;
	bool success=false;
	Eigen::Vector3d err;
	Eigen::VectorXd v(3);
	pinocchio::Data::Matrix6x jac(6, agentParams.pinoModel.nv);

	for (int j = 0; j < max_iter; ++j) {
		pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, q_out);
		pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);

		err = posDes - agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();

		if (err.norm() < tol) {
			if ((agentParams.pinoModel.upperPositionLimit.array() < q_out.array()).any() or
			    (agentParams.pinoModel.lowerPositionLimit.array() > q_out.array()).any()){
				success = false;
			} else success= true;
			break;
		}

		pinocchio::computeFrameJacobian(agentParams.pinoModel, agentParams.pinoData, q_out, agentParams.pinoFrameId,
		                            pinocchio::LOCAL_WORLD_ALIGNED, jac);
		Eigen::MatrixXd jac_pos = jac.topRows(3);
		Eigen::Matrix3d JJt;
		JJt.noalias() = jac_pos * jac_pos.transpose();
		JJt.diagonal().array() += 1e-6;  // Add Damping in Diagonal
		v.noalias() = jac_pos.transpose() * JJt.ldlt().solve(err);
		q_out = q_out + v * 1e-1;
		pinocchio::normalize(agentParams.pinoModel, q_out);
	}
	return success;
}