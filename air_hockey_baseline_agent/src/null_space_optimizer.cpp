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
#include "air_hockey_baseline_agent/null_space_optimizer.h"

using namespace std;
using namespace air_hockey_baseline_agent;
using namespace Eigen;
using namespace iiwas_kinematics;

NullSpaceOptimizer::NullSpaceOptimizer(Kinematics *kinematics, double rate) :
		kinematics_(kinematics),
		stepSize_(1 / rate) {
	solver_.settings()->setWarmStart(true);
	solver_.settings()->setVerbosity(false);
	solver_.data()->setNumberOfVariables(4);
	solver_.data()->setNumberOfConstraints(NUM_OF_JOINTS);
	P_.resize(4, 4);
	A_.resize(7, 4);
	P_.setIdentity();
	A_.setZero();
	dimNullSpace_ = 4;

	if (!solver_.data()->setHessianMatrix(P_)) { cout << "Set Hessian Error!" << endl; }
	if (!solver_.data()->setGradient(q_)) { cout << "Set Gradient Error!" << endl; }
	if (!solver_.data()->setLinearConstraintsMatrix(A_)) { cout << "Set Constraint Matrix Error!" << endl; }
	if (!solver_.data()->setLowerBound(kinematics_->velLimitsLower_)) { cout << "Set Lower Bound Error!" << endl; }
	if (!solver_.data()->setUpperBound(kinematics_->velLimitsUpper_)) { cout << "Set Upper Bound Error!" << endl; }

	solver_.initSolver();

	K_.setConstant(1 / stepSize_);
	weights_ << 10., 10., 5., 10., 1., 1., 0.;
	weightsAnchor_ << 1., 1., 5., 1, 10., 10., 0.;
}

NullSpaceOptimizer::~NullSpaceOptimizer() = default;

bool NullSpaceOptimizer::optimizeJointTrajectory(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                                 const Kinematics::JointArrayType &qStart,
                                                 trajectory_msgs::JointTrajectory &jointTraj) {
	if (!cartTraj.points.empty()) {
		Vector3d xDes, dxDes;
		Kinematics::JointArrayType qCur, qNext, dqNext;

		qCur = qStart;

		trajectory_msgs::JointTrajectoryPoint jointViaPoint_;
		jointViaPoint_.positions.resize(iiwas_kinematics::NUM_OF_JOINTS);
		jointViaPoint_.velocities.resize(iiwas_kinematics::NUM_OF_JOINTS);
		jointViaPoint_.accelerations.resize(iiwas_kinematics::NUM_OF_JOINTS);
		for (int i = 0; i < NUM_OF_JOINTS; ++i) {
			jointViaPoint_.positions[i] = 0;
			jointViaPoint_.velocities[i] = 0;
			jointViaPoint_.accelerations[i] = 0;
		}

		for (size_t i = 0; i < cartTraj.points.size(); ++i) {
			xDes[0] = cartTraj.points[i].transforms[0].translation.x;
			xDes[1] = cartTraj.points[i].transforms[0].translation.y;
			xDes[2] = cartTraj.points[i].transforms[0].translation.z;

			dxDes[0] = cartTraj.points[i].velocities[0].linear.x;
			dxDes[1] = cartTraj.points[i].velocities[0].linear.y;
			dxDes[2] = cartTraj.points[i].velocities[0].linear.z;

			if (!solveQP(xDes, dxDes, qCur, qNext, dqNext)) {
				ROS_INFO_STREAM("Optimization failed at : " << i);
				ROS_INFO_STREAM("xDes: " << xDes.transpose());
				ROS_INFO_STREAM("dxDes: " << dxDes.transpose());
				Vector3d xTmp;
				kinematics_->forwardKinematics(qCur, xTmp);
				ROS_INFO_STREAM("qCur: " << qCur.transpose());
				ROS_INFO_STREAM("xCur: " << xTmp.transpose());
				return false;
			}

			SolveJoint7(qNext, dqNext);

			jointViaPoint_.time_from_start = cartTraj.points[i].time_from_start;
			for (size_t row = 0; row < NUM_OF_JOINTS; row++) {
				jointViaPoint_.accelerations[row] = 0;
				jointViaPoint_.velocities[row] = dqNext[row];
				jointViaPoint_.positions[row] = qNext[row];
			}
			jointTraj.points.push_back(jointViaPoint_);

			qCur = qNext;
		}

		interpolateAcceleration(jointTraj);
		return true;
	} else {
		return false;
	}
}

bool NullSpaceOptimizer::optimizeJointTrajectoryAnchor(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                                       const Kinematics::JointArrayType &qStart,
                                                       const Kinematics::JointArrayType &qAnchor,
                                                       trajectory_msgs::JointTrajectory &jointTraj,
                                                       bool increasing) {

	if (!cartTraj.points.empty()) {
		Vector3d xDes, dxDes;
		Kinematics::JointArrayType qCur, qNext, dqNext, dqAnchorTmp;
		double tTogo, scale;

		qCur = qStart;

		trajectory_msgs::JointTrajectoryPoint jointViaPoint_;
		jointViaPoint_.positions.resize(iiwas_kinematics::NUM_OF_JOINTS);
		jointViaPoint_.velocities.resize(iiwas_kinematics::NUM_OF_JOINTS);
		jointViaPoint_.accelerations.resize(iiwas_kinematics::NUM_OF_JOINTS);
		for (int i = 0; i < NUM_OF_JOINTS; ++i) {
			jointViaPoint_.positions[i] = 0;
			jointViaPoint_.velocities[i] = 0;
			jointViaPoint_.accelerations[i] = 0;
		}

		for (size_t i = 0; i < cartTraj.points.size(); ++i) {
			xDes[0] = cartTraj.points[i].transforms[0].translation.x;
			xDes[1] = cartTraj.points[i].transforms[0].translation.y;
			xDes[2] = cartTraj.points[i].transforms[0].translation.z;

			dxDes[0] = cartTraj.points[i].velocities[0].linear.x;
			dxDes[1] = cartTraj.points[i].velocities[0].linear.y;
			dxDes[2] = cartTraj.points[i].velocities[0].linear.z;

//            tTogo = (cartTraj.points.back().time_from_start - cartTraj.points[i].time_from_start).toSec() + 0.1;
			dqAnchorTmp = (qAnchor - qCur) / 0.5;

			if (increasing) {
				//! Multiply the phase variable
				scale = (cartTraj.points[i].time_from_start.toSec() - cartTraj.points.front().time_from_start.toSec()) /
				        (cartTraj.points.back().time_from_start.toSec() -
				         cartTraj.points.front().time_from_start.toSec());
			} else {
				scale = 1 -
				        (cartTraj.points[i].time_from_start.toSec() - cartTraj.points.front().time_from_start.toSec()) /
				        (cartTraj.points.back().time_from_start.toSec() -
				         cartTraj.points.front().time_from_start.toSec());
			}
			dqAnchorTmp = dqAnchorTmp * scale;

			if (!solveQPAnchor(xDes, dxDes, qCur, dqAnchorTmp, qNext, dqNext)) {
				ROS_INFO_STREAM("Optimization failed at : " << i);
				ROS_INFO_STREAM("xDes: " << xDes.transpose());
				Vector3d xTmp;
				kinematics_->forwardKinematics(qCur, xTmp);
				ROS_INFO_STREAM("qCur: " << qCur.transpose());
				ROS_INFO_STREAM("xCur: " << xTmp.transpose());
				return false;
			}
			SolveJoint7(qNext, dqNext);

			jointViaPoint_.time_from_start = cartTraj.points[i].time_from_start;
			for (size_t row = 0; row < NUM_OF_JOINTS; row++) {
				jointViaPoint_.accelerations[row] = 0.;
				jointViaPoint_.velocities[row] = dqNext[row];
				jointViaPoint_.positions[row] = qNext[row];
			}
			jointTraj.points.push_back(jointViaPoint_);

			qCur = qNext;
		}

		interpolateAcceleration(jointTraj);
		return true;
	} else {
		return false;
	}
}

bool NullSpaceOptimizer::solveQP(const Vector3d &xDes,
                                 const Vector3d &dxDes,
                                 const Kinematics::JointArrayType &qCur,
                                 Kinematics::JointArrayType &qNext,
                                 Kinematics::JointArrayType &dqNext) {
	kinematics_->forwardKinematics(qCur, xCurPos_);

	kinematics_->jacobianPos(qCur, jacobian_);
	getNullSpace(jacobian_, nullSpace_);

	auto b = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() *
	         (K_.asDiagonal() * (xDes - xCurPos_) + dxDes);

	if (nullSpace_.cols() != dimNullSpace_) {
		ROS_INFO_STREAM(qCur.transpose());
		ROS_ERROR_STREAM("Null space of jacobian should be:" << dimNullSpace_);
		return false;
	}

	P_ = (nullSpace_.transpose() * weights_.asDiagonal() * nullSpace_).sparseView();
	q_ = b.transpose() * weights_.asDiagonal() * nullSpace_;
	A_ = nullSpace_.sparseView();

	upperBound_ = kinematics_->velLimitsUpper_.cwiseMin((kinematics_->posLimitsUpper_ * 0.95 - qCur) / stepSize_);
    lowerBound_ = kinematics_->velLimitsLower_.cwiseMax((kinematics_->posLimitsLower_ * 0.95 - qCur) / stepSize_);

	if (!solver_.clearSolverVariables()) { return false; }
	if (!solver_.updateHessianMatrix(P_)) { return false; }
	if (!solver_.updateGradient(q_)) { return false; }
	if (!solver_.updateLinearConstraintsMatrix(A_)) { return false; }
	if (!solver_.updateBounds(lowerBound_ - b, upperBound_ - b)) { return false; }
	if (!solver_.solve()) {
		return false;
	}

	dqNext = b + nullSpace_ * solver_.getSolution();
	qNext = qCur + dqNext * stepSize_;

	return true;
}

bool NullSpaceOptimizer::solveQPAnchor(const Vector3d &xDes,
                                       const Vector3d &dxDes,
                                       const Kinematics::JointArrayType &qCur,
                                       const Kinematics::JointArrayType &dqAnchor,
                                       Kinematics::JointArrayType &qNext,
                                       Kinematics::JointArrayType &dqNext) {
	kinematics_->forwardKinematics(qCur, xCurPos_);

	kinematics_->jacobianPos(qCur, jacobian_);
	getNullSpace(jacobian_, nullSpace_);
//	auto b = jacobian_.completeOrthogonalDecomposition().solve((K_.asDiagonal() * (xDes - xCurPos_) + dxDes));
	auto jacInv = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse();
	auto b = jacInv * (K_.asDiagonal() * (xDes - xCurPos_) + dxDes);
	MatrixXd I(7, 7);
	I.setIdentity();
	auto dqAnchorProjected = (I - jacInv * jacobian_) * dqAnchor;
	auto omega = b - dqAnchorProjected;

	if (nullSpace_.cols() != dimNullSpace_) {
		ROS_ERROR_STREAM("Null space of jacobian should be:" << dimNullSpace_);
		return false;
	}

	P_ = (nullSpace_.transpose() * weightsAnchor_.asDiagonal() * nullSpace_).sparseView();
	q_ = omega.transpose() * weightsAnchor_.asDiagonal() * nullSpace_;
	A_ = nullSpace_.sparseView();

    upperBound_ = kinematics_->velLimitsUpper_.cwiseMin((kinematics_->posLimitsUpper_ * 0.95 - qCur) / stepSize_);
    lowerBound_ = kinematics_->velLimitsLower_.cwiseMax((kinematics_->posLimitsLower_ * 0.95 - qCur) / stepSize_);

	if (!solver_.clearSolverVariables()) { return false; }
	if (!solver_.updateHessianMatrix(P_)) { return false; }
	if (!solver_.updateGradient(q_)) { return false; }
	if (!solver_.updateLinearConstraintsMatrix(A_)) { return false; }
	if (!solver_.updateBounds(lowerBound_ - b, upperBound_ - b)) { return false; }
	if (!solver_.solve()) {
		return false;
	}

	dqNext = b + nullSpace_ * solver_.getSolution();
	qNext = qCur + dqNext * stepSize_;

	return true;
}

void NullSpaceOptimizer::SolveJoint7(JointArrayType &q, JointArrayType &dq) {
	double qCur7 = q[6];
	// Set last joint to 0 solve desired joint position
	q[6] = 0.0;
	Vector3d eePos;
	Quaterniond eeQuat;
	kinematics_->forwardKinematics(q, eePos, eeQuat);
	Matrix3d mat = eeQuat.toRotationMatrix();
	Vector3d zAxis(0., 0., -1.);
	auto yDes = zAxis.cross(mat.col(2)).normalized();
	double target = acos(mat.col(1).dot(yDes));
	Vector3d axis = mat.col(1).cross(yDes).normalized();
	target = target * axis.dot(mat.col(2));

	if (target - qCur7 > M_PI_2) {
		target -= M_PI;
	} else if (target - qCur7 < M_PI_2) {
		target += M_PI;
	}

	q[6] = boost::algorithm::clamp(target, kinematics_->posLimitsLower_[6], kinematics_->posLimitsUpper_[6]);
	dq[6] = (q[6] - qCur7) / stepSize_;
}

void NullSpaceOptimizer::interpolateAcceleration(trajectory_msgs::JointTrajectory &jointTraj) {
	for (int i = 1; i < jointTraj.points.size() - 1; ++i) {
		for (int j = 0; j < NUM_OF_JOINTS; ++j) {
			jointTraj.points[i].accelerations[j] =
					(jointTraj.points[i + 1].velocities[j] - jointTraj.points[i - 1].velocities[j]) / (2 * stepSize_);
		}
	}

}
