//
// Created by puze on 06.12.20.
//
#include "iostream"
#include "null_space_opt.h"

using namespace std;
using namespace null_space_optim;

NullSpaceOptimizer::NullSpaceOptimizer(Kinematics &kinematics) : solver_(), kinematics_(kinematics) {
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
    solver_.data()->setNumberOfVariables(4);
    solver_.data()->setNumberOfConstraints(NUM_OF_JOINTS);

    P_.resize(4, 4);
    A_.resize(7, 4);
    P_.setIdentity();
    A_.setZero();
    dim_null_space_ = 4;

    if (!solver_.data()->setHessianMatrix(P_)) { cout << "Set Hessian Error!" << endl; }
    if (!solver_.data()->setGradient(q_)) { cout << "Set Gradient Error!" << endl; }
    if (!solver_.data()->setLinearConstraintsMatrix(A_)) { cout << "Set Constraint Matrix Error!" << endl; }
    if (!solver_.data()->setLowerBound(kinematics_.vel_limits_lower)) { cout << "Set Lower Bound Error!" << endl; }
    if (!solver_.data()->setUpperBound(kinematics_.vel_limits_upper)) { cout << "Set Upper Bound Error!" << endl; }

    solver_.initSolver();

    K_ << 10., 10., 10.;
}

void NullSpaceOptimizer::GetNullSpace(const NullSpaceOptimizer::JacobianPosType &jacobian,
                                      MatrixXd &out_null_space,
                                      int &out_rank) {
    CompleteOrthogonalDecomposition<Matrix<double, Dynamic, Dynamic> > cod;
    cod.compute(jacobian);
    out_rank = cod.dimensionOfKernel();
    // Find URV^T
    MatrixXd V = cod.matrixZ().transpose();
    out_null_space = V.block(0, cod.rank(), V.rows(), V.cols() - cod.rank());
//    MatrixXd P = cod.colsPermutation();
    out_null_space = cod.colsPermutation() * out_null_space; // Unpermute the columns
}

void NullSpaceOptimizer::ConstructQP(const Vector3d &x_des,
                                     const Vector3d &dx_des,
                                     const Kinematics::JointArrayType &q_cur,
                                     const Kinematics::JointArrayType &weights) {
    kinematics_.ForwardKinematics(q_cur, x_cur_pos_, x_cur_quat_);
    kinematics_.JacobianPos(q_cur, jacobian_);
    int num_of_variables;
    GetNullSpace(jacobian_, null_space_, num_of_variables);

    auto x_err_pos_ = K_.asDiagonal() * (x_des - x_cur_pos_);
    auto b = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse() * (x_err_pos_ + dx_des);

    if (num_of_variables != dim_null_space_){
        throw std::runtime_error(std::string("The Jacobian is at Singularity, dimension is: ") + to_string(num_of_variables));
    }

    P_ = (null_space_.transpose() * weights.asDiagonal() * null_space_).sparseView();
    q_ = b.transpose() * weights.asDiagonal() * null_space_;
    A_ = null_space_.sparseView();

    solver_.updateHessianMatrix(P_);
    solver_.updateGradient(q_);
    solver_.updateLinearConstraintsMatrix(A_);
    solver_.updateBounds(kinematics_.vel_limits_lower - b, kinematics_.vel_limits_upper - b);
    solver_.solve();

//    cout << "Solved: " << solver_.solve() << endl;
    cout << "Solution: " << solver_.getSolution() << endl;
}

void NullSpaceOptimizer::castConstraintMatrix(SparseMatrix<double> &constraint_matrix, const int &num_of_variables) {
    constraint_matrix.resize(NUM_OF_JOINTS * 2, num_of_variables);
    for (int i = 0; i < NUM_OF_JOINTS; ++i) {
        constraint_matrix.insert(i, i) = 1.0;
        constraint_matrix.insert(i + 7, i + 7) = -1;
    }
}


