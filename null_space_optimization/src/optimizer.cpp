//
// Created by puze on 06.12.20.
//
#include "iostream"
#include "optimizer.h"

using namespace std;
using namespace null_space_optimization;

NullSpaceOptimizer::NullSpaceOptimizer(Kinematics &kinematics) : solver_(), kinematics_(kinematics) {
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
    if (!solver_.data()->setLowerBound(kinematics_.velLimitsLower_)) { cout << "Set Lower Bound Error!" << endl; }
    if (!solver_.data()->setUpperBound(kinematics_.velLimitsUpper_)) { cout << "Set Upper Bound Error!" << endl; }

    solver_.initSolver();

    K_ << 10., 10., 10.;
}

void NullSpaceOptimizer::GetNullSpace(const NullSpaceOptimizer::JacobianPosType &jacobian,
                                      MatrixXd &out_null_space) {
    CompleteOrthogonalDecomposition<Matrix<double, Dynamic, Dynamic> > cod;
    cod.compute(jacobian);
    num_of_variables_ = cod.dimensionOfKernel();
    // Find URV^T
    V_ = cod.matrixZ().transpose();
    out_null_space = V_.block(0, cod.rank(), V_.rows(), V_.cols() - cod.rank());
//    MatrixXd P = cod.colsPermutation();
    out_null_space = cod.colsPermutation() * out_null_space; // Unpermute the columns
}

bool NullSpaceOptimizer::solveQP(const Vector3d &xDes,
                                 const Vector3d &dxDes,
                                 const Kinematics::JointArrayType &qCur,
                                 const Kinematics::JointArrayType &weights,
                                 const double stepSize,
                                 Kinematics::JointArrayType &qNext,
                                 Kinematics::JointArrayType &dqNext) {
    kinematics_.forwardKinematics(qCur, xCurPos_, xCurQuat_);

    if ((xCurPos_ - xDes).norm() / stepSize > 2){
        cout << "Optimization failed: the current position is too far from desired position" << endl;
        return false;
    }
    kinematics_.jacobianPos(qCur, jacobian_);
    GetNullSpace(jacobian_, nullSpace_);

    auto x_err_pos_ = K_.asDiagonal() * (xDes - xCurPos_);
    auto b = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * (x_err_pos_ + dxDes);

    if (num_of_variables_ != dimNullSpace_) {
        throw std::runtime_error(
                std::string("The Jacobian is at Singularity, dimension is: ") + to_string(num_of_variables_));
    }

    P_ = (nullSpace_.transpose() * weights.asDiagonal() * nullSpace_).sparseView();
    q_ = b.transpose() * weights.asDiagonal() * nullSpace_;
    A_ = nullSpace_.sparseView();

    if (!solver_.clearSolverVariables()) { return false; }
    if (!solver_.updateHessianMatrix(P_)) { return false; }
    if (!solver_.updateGradient(q_)) { return false; }
    if (!solver_.updateLinearConstraintsMatrix(A_)) { return false; }
    if (!solver_.updateBounds(kinematics_.velLimitsLower_ - b, kinematics_.velLimitsUpper_ - b)) { return false; }
    if (!solver_.solve()) { return false; }

    dqNext = b + nullSpace_ * solver_.getSolution();
    qNext = qCur + dqNext * stepSize;

    return true;
}


