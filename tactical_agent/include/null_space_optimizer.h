#ifndef SRC_NULL_SPACE_OPTIMIZER_H
#define SRC_NULL_SPACE_OPTIMIZER_H

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>

#include "iiwas_kinematics.h"



namespace tactical_agent{
class NullSpaceOptimizer {
public:
    NullSpaceOptimizer(iiwas_kinematics::Kinematics kinematics, bool closeLoop=false);
    ~NullSpaceOptimizer();

    bool ProcessTrajectory(trajectory_msgs::MultiDOFJointTrajectory cartTraj);

private:
    bool solveQP(const Vector3d &xDes,
                 const Vector3d &dxDes,
                 const iiwas_kinematics::Kinematics::JointArrayType &qCur,
                 const iiwas_kinematics::Kinematics::JointArrayType &weights,
                 const double timeStep,
                 iiwas_kinematics::Kinematics::JointArrayType &qNext,
                 iiwas_kinematics::Kinematics::JointArrayType &dqNext);
    void GetNullSpace(const iiwas_kinematics::Kinematics::JacobianPosType &jacobian, MatrixXd &out_null_space);

private:
    bool closeLoop_;

    iiwas_kinematics::Kinematics kinematics_;

    OsqpEigen::Solver solver_;
    int dimNullSpace_;
    SparseMatrix<double> P_;
    Matrix<double, 4, 1> q_;
    SparseMatrix<double> A_;
};
}


#endif //SRC_NULL_SPACE_OPTIMIZER_H
