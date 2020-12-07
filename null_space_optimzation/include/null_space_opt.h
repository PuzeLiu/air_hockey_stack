//
// Created by puze on 06.12.20.
//

#ifndef SRC_NULL_SPACE_OPT_HPP
#define SRC_NULL_SPACE_OPT_HPP

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include "iiwas_kinematics.h"

using namespace iiwas_kinematics;

namespace null_space_optim {
    class NullSpaceOptimizer {
    public:
        typedef Kinematics::JacobianPosType JacobianPosType;
    public:
        NullSpaceOptimizer(Kinematics &kinematics);

        int SolveQP(const Vector3d &x_des,
                         const Vector3d &dx_des,
                         const Kinematics::JointArrayType &q_cur,
                         const Kinematics::JointArrayType &weights);

        void GetNullSpace(const JacobianPosType& jacobian, MatrixXd &out_null_space, int &out_rank);

    private:
        void castConstraintMatrix(Eigen::SparseMatrix<double> &constraintMatrix, const int& num_of_variables);

    private:
        Kinematics &kinematics_;
        Kinematics::JacobianPosType jacobian_;
        MatrixXd null_space_;

        Vector3d K_; // Weight for correcting position error

        Vector3d x_cur_pos_;
        Quaterniond x_cur_quat_;

        OsqpEigen::Solver solver_;
        int dim_null_space_;
        SparseMatrix<double> P_;
        Matrix<double, 4, 1> q_;
        SparseMatrix<double> A_;
        Matrix<double, 7, 1> lb_, ub_;

    };
}


#endif //SRC_NULL_SPACE_OPT_HPP
