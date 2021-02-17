//
// Created by puze on 11.02.21.
//
#include <iostream>
#include "air_hockey_baseline_agent/hitting_point_optimizer.h"
#include <chrono>
#include <coin/CoinPackedVector.hpp>


using namespace std;
using namespace nlopt;
using namespace Eigen;
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

OptimizerData::OptimizerData(Kinematics &kinematics) : kinematics(kinematics) {
    epsilon = sqrt(numeric_limits<double>::epsilon());
}

HittingPointOptimizer::HittingPointOptimizer(Kinematics &kinematics) : optData(kinematics) {
    nlSolver = opt(LD_SLSQP, iiwas_kinematics::NUM_OF_JOINTS);
    nlSolver.set_max_objective(objective, &optData);

    double tol = 1e-4;
    nlSolver.add_equality_constraint(this->equalityConstraint, &optData, tol);

    std::vector<double> qLower(optData.kinematics.posLimitsLower_.data(),
                               optData.kinematics.posLimitsLower_.data() +
                               optData.kinematics.posLimitsLower_.cols() * optData.kinematics.posLimitsLower_.rows());
    nlSolver.set_lower_bounds(qLower);

    std::vector<double> qUpper(optData.kinematics.posLimitsUpper_.data(),
                               optData.kinematics.posLimitsUpper_.data() +
                               optData.kinematics.posLimitsUpper_.cols() * optData.kinematics.posLimitsUpper_.rows());
    nlSolver.set_upper_bounds(qUpper);

    // Set tolerance for nonlinear solver
    nlSolver.set_ftol_abs(1e-4);
    nlSolver.set_xtol_abs(1e-6);

    // Set up linear programming solver
    simplexModel.setLogLevel(0);
}

HittingPointOptimizer::~HittingPointOptimizer() {

}

bool HittingPointOptimizer::solve(const Eigen::Vector3d &hitPoint, const Eigen::Vector3d &hitDirection,
                                  iiwas_kinematics::Kinematics::JointArrayType &qInOut, double &velMagMax) {
    optData.hitPoint = hitPoint;
    optData.hitDirection = hitDirection;

    if (!getInitPoint(qInOut)) {
        return false;
    }

    std::vector<double> qCur(qInOut.data(), qInOut.data() + qInOut.rows() * qInOut.cols());
    std::vector<double> grad(7);

    double opt_fun;
    auto result = nlSolver.optimize(qCur, opt_fun);

    if (result < 0) {
        return false;
    }

    if (h(qCur, &optData) < 1e-4) {
        for (int i = 0; i < qCur.size(); ++i) {
            qInOut[i] = qCur[i];
        }

//        velMagMax = getMaxVelocity(qInOut);
        velMagMax = getMaxVelocityLP(qInOut);
        return true;
    } else {
        cout << "The position error is : " << h(qCur, &optData) << " bigger than 1e-4" << endl;
        cout << "termination condition:" << nlopt_result_to_string(nlopt_result(result)) << endl;
        return false;
    }
}

double HittingPointOptimizer::objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data) {
    auto optData = (OptimizerData *) f_data;
    if (!grad.empty()) {
        numerical_grad(f, x, optData, grad);
    }
    return f(x, optData);
}

double HittingPointOptimizer::equalityConstraint(const vector<double> &x, vector<double> &grad, void *f_data) {
    auto optData = (OptimizerData *) f_data;
    if (!grad.empty()) {
        numerical_grad(h, x, optData, grad);
    }
    return h(x, optData);
}

double HittingPointOptimizer::f(const vector<double> &x, const OptimizerData *data) {
    iiwas_kinematics::Kinematics::JointArrayType qCur(x.data());
    iiwas_kinematics::Kinematics::JacobianPosType jacobian;
    data->kinematics.jacobianPos(qCur, jacobian);

    auto vec = data->hitDirection.transpose() * jacobian;
    return vec.squaredNorm();
}

double HittingPointOptimizer::h(const vector<double> &x, const OptimizerData *data) {
    iiwas_kinematics::Kinematics::JointArrayType qCur(x.data());
    Eigen::Vector3d xCur;
    data->kinematics.forwardKinematics(qCur, xCur);

    auto distance = (xCur - data->hitPoint).norm();
    return distance;
}

void HittingPointOptimizer::numerical_grad(HittingPointOptimizer::functype function, const vector<double> &x,
                                           const OptimizerData *data, vector<double> &grad) {
    vector<double> x_pos, x_neg;
    for (int i = 0; i < x.size(); ++i) {
        x_pos = x;
        x_neg = x;
        x_pos[i] += data->epsilon;
        x_neg[i] -= data->epsilon;
        grad[i] = (function(x_pos, data) - function(x_neg, data)) / (2 * data->epsilon);
    }
}

bool HittingPointOptimizer::getInitPoint(iiwas_kinematics::Kinematics::JointArrayType &qInOut) {
    if (!optData.kinematics.numericalInverseKinematics(optData.hitPoint, qInOut, 1e-4, 200)) {
        cout << "No feasible IK solution" << endl;
        return false;
    }
    return true;
}

double HittingPointOptimizer::getMaxVelocity(const iiwas_kinematics::Kinematics::JointArrayType &q) {
    Kinematics::JacobianPosType jac;
    optData.kinematics.jacobianPos(q, jac);
    auto jacInv = jac.transpose() * (jac * jac.transpose()).inverse();
    auto qRef = jacInv * optData.hitDirection;

    auto min_scale = optData.kinematics.velLimitsUpper_.cwiseProduct(qRef.cwiseAbs().cwiseInverse()).minCoeff();
    auto q_res = (jacInv * (min_scale * optData.hitDirection));
//    cout << "Pseudo inverse of Jacobian" << endl;
//    cout << "##########################" << endl;
//    cout << q_res.transpose() << endl;
//    cout << "MaxVel: " << min_scale << endl;
    return min_scale;
}

double HittingPointOptimizer::getMaxVelocityLP(const Kinematics::JointArrayType &q) {
    Kinematics::JacobianPosType jac;
    optData.kinematics.jacobianPos(q, jac);
    MatrixXd orthogonalComplement;
    getNullSpace(optData.hitDirection.transpose(), orthogonalComplement);

    MatrixXd OCJac = (orthogonalComplement.transpose() * jac);
    MatrixXd OCJacNullSpace;
    getNullSpace(OCJac, OCJacNullSpace);

    // Get Objective
    MatrixXd objective = -optData.hitDirection.transpose() * jac * OCJacNullSpace;

    // Get Bounds for Primal Variables
    int nRows = OCJacNullSpace.rows();
    int nCols = OCJacNullSpace.cols();
    VectorXd columnLower(nCols);
    VectorXd columnUpper(nCols);
    columnLower = -columnLower.setOnes() * COIN_DBL_MAX;
    columnUpper = columnUpper.setOnes() * COIN_DBL_MAX;

    // Get Constraint Matrix
    CoinPackedMatrix matrix;
    matrix.setDimensions(nRows, nCols);
    for (int i = 0; i < OCJacNullSpace.rows(); ++i) {
        for (int j = 0; j < OCJacNullSpace.cols(); ++j) {
            matrix.modifyCoefficient(i, j, OCJacNullSpace(i, j));
        }
    }

    // Generate Problem
    simplexModel.loadProblem(matrix, columnLower.data(), columnUpper.data(), objective.data(),
                             optData.kinematics.velLimitsLower_.data(), optData.kinematics.velLimitsUpper_.data());

    // Get Solution
    simplexModel.dual();

//    VectorXd solution(nCols);
//    auto solutionConst = simplexModel.getColSolution();
//    for (int i = 0; i < nCols; ++i) {
//        solution[i] = solutionConst[i];
//    }
//    cout << "    Linear Programming    " << endl;
//    cout << "##########################" << endl;
//    cout << solution.transpose() << endl;
//    cout << "Constraint: " << (OCJacNullSpace * solution).transpose() << endl;
//    cout << "Limit: " << optData.kinematics.velLimitsUpper_.transpose() << endl;
//    cout << "Obj: " << optData.hitDirection.transpose() * jac * OCJacNullSpace * solution << endl;
//
//    cout << "          Problem         " << endl;
//    cout << "##########################" << endl;
//    cout << (optData.hitDirection.transpose() * jac * OCJacNullSpace) << endl;
//    cout << "        Constraints       " << endl;
//    cout << OCJacNullSpace << endl;

    return -simplexModel.getObjValue();
}


