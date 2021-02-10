//
// Created by puze on 06.12.20.
//

#ifndef SRC_NULL_SPACE_OPT_HPP
#define SRC_NULL_SPACE_OPT_HPP

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include "iiwas_kinematics/iiwas_kinematics.h"

namespace null_space_optimization {
    class NullSpaceOptimizer {
    public:
        typedef iiwas_kinematics::Kinematics::JacobianPosType JacobianPosType;
        typedef iiwas_kinematics::Kinematics::JointArrayType JointArrayType;

    public:
        NullSpaceOptimizer(iiwas_kinematics::Kinematics &kinematics);

        bool solveQP(const Eigen::Vector3d &xDes,
                     const Eigen::Vector3d &dxDes,
                     const JointArrayType &qCur,
                     const JointArrayType &weights,
                     const double timeStep,
                     JointArrayType &qNext,
                     JointArrayType &dqNext);

    private:
        void GetNullSpace(const JacobianPosType &jacobian, Eigen::MatrixXd &out_null_space);

    public:
        iiwas_kinematics::Kinematics &kinematics_;
        JacobianPosType jacobian_;
        Eigen::MatrixXd nullSpace_;

    private:
        int num_of_variables_;  //Dimensions of jacobian
        Eigen::Vector3d K_;            // Weight for correcting position error

        Eigen::Vector3d xCurPos_;
        Eigen::Quaterniond xCurQuat_;

        OsqpEigen::Solver solver_;
        int dimNullSpace_;
        Eigen::SparseMatrix<double> P_;
        Eigen::Matrix<double, 4, 1> q_;
        Eigen::SparseMatrix<double> A_;

        Eigen::MatrixXd V_;            // temporal matrix for calculating null space
    };
}


#endif //SRC_NULL_SPACE_OPT_HPP
