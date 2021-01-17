#ifndef SRC_NULL_SPACE_OPTIMIZER_H
#define SRC_NULL_SPACE_OPTIMIZER_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>

#include "observer.h"
#include "iiwas_kinematics.h"

using namespace iiwas_kinematics;

namespace AirHockey {
    class NullSpaceOptimizer {
    public:
        NullSpaceOptimizer(iiwas_kinematics::Kinematics *kinematics, Observer *observer, bool closeLoop = false,
                           double rate = 100.);

        ~NullSpaceOptimizer();

        bool optimizeJointTrajectory(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                     const Kinematics::JointArrayType &qStart,
                                     trajectory_msgs::JointTrajectory &jointTraj);

        bool optimizeJointTrajectoryAnchor(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                           const Kinematics::JointArrayType &qStart,
                                           const Kinematics::JointArrayType &qAnchor,
                                           trajectory_msgs::JointTrajectory &jointTraj);

    private:
        bool solveQP(const Vector3d &xDes,
                     const Vector3d &dxDes,
                     const Kinematics::JointArrayType &qCur,
                     Kinematics::JointArrayType &qNext,
                     Kinematics::JointArrayType &dqNext);

        bool solveQPAnchor(const Vector3d &xDes,
                           const Vector3d &dxDes,
                           const Kinematics::JointArrayType &qCur,
                           const Kinematics::JointArrayType &qAnchor,
                           Kinematics::JointArrayType &qNext,
                           Kinematics::JointArrayType &dqNext);

        void GetNullSpace(const iiwas_kinematics::Kinematics::JacobianPosType &jacobian, MatrixXd &out_null_space);

    private:
        bool closeLoop_;
        double stepSize_;

        iiwas_kinematics::Kinematics *kinematics_;
        Observer *observer_;

        trajectory_msgs::JointTrajectoryPoint jointViaPoint_;

        OsqpEigen::Solver solver_;
        int dimNullSpace_;
        SparseMatrix<double> P_;
        Matrix<double, 4, 1> q_;
        SparseMatrix<double> A_;
        Vector3d K_;            // Weight for correcting position error
        MatrixXd V_;            // temporal matrix for calculating null space

        Vector3d xCurPos_;
        iiwas_kinematics::Kinematics::JacobianPosType jacobian_;
        MatrixXd nullSpace_;

        iiwas_kinematics::Kinematics::JointArrayType weights_;
    };
}


#endif //SRC_NULL_SPACE_OPTIMIZER_H
