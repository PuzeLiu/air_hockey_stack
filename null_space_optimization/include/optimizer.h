//
// Created by puze on 06.12.20.
//

#ifndef SRC_NULL_SPACE_OPT_HPP
#define SRC_NULL_SPACE_OPT_HPP

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include "iiwas_kinematics.h"

using namespace iiwas_kinematics;

namespace null_space_optimization {
    class NullSpaceOptimizer {
    public:
        typedef Kinematics::JacobianPosType JacobianPosType;
    public:
        NullSpaceOptimizer(Kinematics &kinematics);

        bool solveQP(const Vector3d &xDes,
                     const Vector3d &dxDes,
                     const Kinematics::JointArrayType &qCur,
                     const Kinematics::JointArrayType &weights,
                     const double timeStep,
                     Kinematics::JointArrayType &qNext,
                     Kinematics::JointArrayType &dqNext);

    private:
        void GetNullSpace(const JacobianPosType &jacobian, MatrixXd &out_null_space);

    public:
        Kinematics &kinematics_;
        Kinematics::JacobianPosType jacobian_;
        MatrixXd nullSpace_;

    private:
        int num_of_variables_;  //Dimensions of jacobian
        Vector3d K_;            // Weight for correcting position error

        Vector3d xCurPos_;
        Quaterniond xCurQuat_;

        OsqpEigen::Solver solver_;
        int dimNullSpace_;
        SparseMatrix<double> P_;
        Matrix<double, 4, 1> q_;
        SparseMatrix<double> A_;

        MatrixXd V_;            // temporal matrix for calculating null space
    };
}


#endif //SRC_NULL_SPACE_OPT_HPP
