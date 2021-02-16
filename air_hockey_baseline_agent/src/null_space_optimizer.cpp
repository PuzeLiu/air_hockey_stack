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

NullSpaceOptimizer::NullSpaceOptimizer(Kinematics *kinematics,
                                       Observer *observer,
                                       bool closeLoop,
                                       double rate) :
        kinematics_(kinematics),
        observer_(observer),
        closeLoop_(closeLoop),
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

    jointViaPoint_.positions.resize(iiwas_kinematics::NUM_OF_JOINTS);
    jointViaPoint_.velocities.resize(iiwas_kinematics::NUM_OF_JOINTS);

    K_.setConstant(1 / stepSize_);
    weights_ << 40., 40., 20., 40., 10., 20., 0.;
    weightsAnchor_.setOnes();
    beta_ = 10;
}

NullSpaceOptimizer::~NullSpaceOptimizer() {

}

bool NullSpaceOptimizer::optimizeJointTrajectory(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                                 const Kinematics::JointArrayType &qStart,
                                                 trajectory_msgs::JointTrajectory &jointTraj) {
    if (cartTraj.points.size() > 0) {
        Vector3d xDes, dxDes;
        Kinematics::JointArrayType qCur, qNext, dqNext;

        qCur = qStart;

        for (size_t i = 0; i < cartTraj.points.size(); ++i) {
            xDes[0] = cartTraj.points[i].transforms[0].translation.x;
            xDes[1] = cartTraj.points[i].transforms[0].translation.y;
            xDes[2] = cartTraj.points[i].transforms[0].translation.z;

            dxDes[0] = cartTraj.points[i].velocities[0].linear.x;
            dxDes[1] = cartTraj.points[i].velocities[0].linear.y;
            dxDes[2] = cartTraj.points[i].velocities[0].linear.z;

            if (!solveQP(xDes, dxDes, qCur, qNext, dqNext)) {
                return false;
            }

            SolveJoint7(qNext);

            jointViaPoint_.time_from_start = cartTraj.points[i].time_from_start;
            for (size_t row = 0; row < NUM_OF_JOINTS; row++) {
                jointViaPoint_.positions[row] = qNext[row];
                jointViaPoint_.velocities[row] = dqNext[row];
            }
            jointTraj.points.push_back(jointViaPoint_);

            qCur = qNext;
        }
        return true;
    } else {
        return false;
    }
}

bool NullSpaceOptimizer::optimizeJointTrajectoryAnchor(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                                       const Kinematics::JointArrayType &qStart,
                                                       const Kinematics::JointArrayType &qAnchor,
                                                       trajectory_msgs::JointTrajectory &jointTraj) {
    if (cartTraj.points.size() > 0) {
        Vector3d xDes, dxDes;
        Kinematics::JointArrayType qCur, qNext, dqNext, dqAnchorTmp;
        double tTogo;

        qCur = qStart;

        for (size_t i = 0; i < cartTraj.points.size(); ++i) {
            xDes[0] = cartTraj.points[i].transforms[0].translation.x;
            xDes[1] = cartTraj.points[i].transforms[0].translation.y;
            xDes[2] = cartTraj.points[i].transforms[0].translation.z;

            dxDes[0] = cartTraj.points[i].velocities[0].linear.x;
            dxDes[1] = cartTraj.points[i].velocities[0].linear.y;
            dxDes[2] = cartTraj.points[i].velocities[0].linear.z;

            tTogo = (cartTraj.points.back().time_from_start - cartTraj.points[i].time_from_start).toSec();
            if (tTogo > 0) {
                dqAnchorTmp = (qAnchor - qCur) / tTogo;
            } else {
                dqAnchorTmp.setZero();
            }

            if (!solveQPAnchor(xDes, dxDes, qCur, dqAnchorTmp, qNext, dqNext)) {
                ROS_INFO_STREAM("Failed Anchor Index: " << i);
                return false;
            }

            SolveJoint7(qNext);

            jointViaPoint_.time_from_start = cartTraj.points[i].time_from_start;
            for (size_t row = 0; row < NUM_OF_JOINTS; row++) {
                jointViaPoint_.positions[row] = qNext[row];
                jointViaPoint_.velocities[row] = dqNext[row];
            }
            jointTraj.points.push_back(jointViaPoint_);

            qCur = qNext;
        }
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
    GetNullSpace(jacobian_, nullSpace_);

    auto b = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() *
             (K_.asDiagonal() * (xDes - xCurPos_) + dxDes);

    if (nullSpace_.cols() != dimNullSpace_) {
        ROS_ERROR_STREAM("Null space of jacobian should be:" << dimNullSpace_);
        return false;
    }

    P_ = (nullSpace_.transpose() * weights_.asDiagonal() * nullSpace_).sparseView();
    q_ = b.transpose() * weights_.asDiagonal() * nullSpace_;
    A_ = nullSpace_.sparseView();

    if (!solver_.clearSolverVariables()) { return false; }
    if (!solver_.updateHessianMatrix(P_)) { return false; }
    if (!solver_.updateGradient(q_)) { return false; }
    if (!solver_.updateLinearConstraintsMatrix(A_)) { return false; }
    if (!solver_.updateBounds(kinematics_->velLimitsLower_ - b, kinematics_->velLimitsUpper_ - b)) { return false; }
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
    GetNullSpace(jacobian_, nullSpace_);

    auto b = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() *
             (K_.asDiagonal() * (xDes - xCurPos_) + dxDes);
    auto omega = b - dqAnchor;

    if (nullSpace_.cols() != dimNullSpace_) {
        ROS_ERROR_STREAM("Null space of jacobian should be:" << dimNullSpace_);
        return false;
    }

    P_ = (nullSpace_.transpose() * weightsAnchor_.asDiagonal() * nullSpace_).sparseView();
    q_ = omega.transpose() * weightsAnchor_.asDiagonal() * nullSpace_;
    A_ = nullSpace_.sparseView();

    if (!solver_.clearSolverVariables()) { return false; }
    if (!solver_.updateHessianMatrix(P_)) { return false; }
    if (!solver_.updateGradient(q_)) { return false; }
    if (!solver_.updateLinearConstraintsMatrix(A_)) { return false; }
    if (!solver_.updateBounds(kinematics_->velLimitsLower_ - b, kinematics_->velLimitsUpper_ - b)) { return false; }
    if (!solver_.solve()) {
        return false;
    }

    dqNext = b + nullSpace_ * solver_.getSolution();
    qNext = qCur + dqNext * stepSize_;

    return true;
}

void NullSpaceOptimizer::SolveJoint7(Kinematics::JointArrayType &q) {
    double qCur = q[6];

    // Set last joint to 0 solve desired joint position
    q[6] = 0.0;
    Vector3d eePos;
    Quaterniond eeQuat;
    kinematics_->forwardKinematics(q, eePos, eeQuat);
    Matrix3d mat = eeQuat.toRotationMatrix();
    Vector3d zAxis(0., 0., 1.);
    auto yDes = zAxis.cross(mat.col(2)).normalized();
    double angle = acos(mat.col(1).dot(yDes));

    if (angle >= M_PI_2) {
        angle -= M_PI;
    }

//    auto dq = boost::algorithm::clamp((angle - qCur)/stepSize_ ,
//                                 kinematics_->velLimitsLower_[6] * 0.8,
//                                 kinematics_->velLimitsUpper_[6]) * 0.8;
//    q[6] = qCur + dq * stepSize_;
    q[6] = angle;
}
