#include "null_space_optimizer.h"

using namespace AirHockey;
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

    K_ << 10., 10., 10.;
    weights_ << 40., 40., 20., 40., 10., 20., 1.;
    weightsAnchor_.setOnes();
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
        Kinematics::JointArrayType qCur, qNext, dqNext;

        qCur = qStart;

        for (size_t i = 0; i < cartTraj.points.size(); ++i) {
            xDes[0] = cartTraj.points[i].transforms[0].translation.x;
            xDes[1] = cartTraj.points[i].transforms[0].translation.y;
            xDes[2] = cartTraj.points[i].transforms[0].translation.z;

            dxDes[0] = cartTraj.points[i].velocities[0].linear.x;
            dxDes[1] = cartTraj.points[i].velocities[0].linear.y;
            dxDes[2] = cartTraj.points[i].velocities[0].linear.z;

            if (!solveQPAnchor(xDes, dxDes, qCur, qAnchor, qNext, dqNext)) {
                ROS_INFO_STREAM("Failed Anchor Index: " << i);
                return false;
            }

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

    auto x_err_pos_ = K_.asDiagonal() * (xDes - xCurPos_);
    auto b = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * (x_err_pos_ + dxDes);

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
                                       const Kinematics::JointArrayType &qAnchor,
                                       Kinematics::JointArrayType &qNext,
                                       Kinematics::JointArrayType &dqNext) {
    kinematics_->forwardKinematics(qCur, xCurPos_);

    kinematics_->jacobianPos(qCur, jacobian_);
    GetNullSpace(jacobian_, nullSpace_);

    auto x_err_pos_ = K_.asDiagonal() * (xDes - xCurPos_);
    auto b = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * (x_err_pos_ + dxDes);
    auto omega = qCur - qAnchor + b * stepSize_;

    if (nullSpace_.cols() != dimNullSpace_) {
        ROS_ERROR_STREAM("Null space of jacobian should be:" << dimNullSpace_);
        return false;
    }

    P_ = (nullSpace_.transpose() * (weightsAnchor_ * pow(stepSize_, 2)).asDiagonal() * nullSpace_).sparseView();
    q_ = omega.transpose() * (weightsAnchor_ * stepSize_).asDiagonal() * nullSpace_;
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

void NullSpaceOptimizer::GetNullSpace(const Kinematics::JacobianPosType &jacobian,
                                      MatrixXd &out_null_space) {
    CompleteOrthogonalDecomposition<Matrix<double, Dynamic, Dynamic> > cod;
    cod.compute(jacobian);
    // Find URV^T
    V_ = cod.matrixZ().transpose();
    out_null_space = V_.block(0, cod.rank(), V_.rows(), V_.cols() - cod.rank());
//    MatrixXd P = cod.colsPermutation();
    out_null_space = cod.colsPermutation() * out_null_space; // Unpermute the columns
}

