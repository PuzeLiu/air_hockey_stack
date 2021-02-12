//
// Created by puze on 11.02.21.
//
#include <iostream>
#include "hitting_point_optimizer.h"
#include <chrono>

using namespace std;
using namespace nlopt;
using namespace iiwas_kinematics;

OptimizerData::OptimizerData(Kinematics& kinematics): kinematics(kinematics) {
    epsilon = sqrt(numeric_limits<double>::epsilon());
}

HittingPointOptimizer::HittingPointOptimizer(Kinematics& kinematics): optData(kinematics) {
    optimizer = opt(LD_SLSQP, iiwas_kinematics::NUM_OF_JOINTS);
    optimizer.set_max_objective(objective, &optData);

    double tol = 1e-8;
    optimizer.add_equality_constraint(this->equalityConstraint, &optData, tol);

    std::vector<double> qLower(optData.kinematics.posLimitsLower_.data(),
                             optData.kinematics.posLimitsLower_.data() +
                             optData.kinematics.posLimitsLower_.cols() * optData.kinematics.posLimitsLower_.rows());
    optimizer.set_lower_bounds(qLower);

    std::vector<double> qUpper(optData.kinematics.posLimitsUpper_.data(),
                               optData.kinematics.posLimitsUpper_.data() +
                               optData.kinematics.posLimitsUpper_.cols() * optData.kinematics.posLimitsUpper_.rows());
    optimizer.set_upper_bounds(qUpper);


    optimizer.set_ftol_rel(1e-6);
//    optimizer.set_ftol_abs(1e-6);
    optimizer.set_xtol_abs(1e-4);
}

HittingPointOptimizer::~HittingPointOptimizer(){

}

bool HittingPointOptimizer::optimize(const Eigen::Vector3d& hitPoint, const Eigen::Vector3d& hitDirection,
                                     const iiwas_kinematics::Kinematics::JointArrayType& qStart,
                                     iiwas_kinematics::Kinematics::JointArrayType& qOut) {
    optData.hitPoint = hitPoint;
    optData.hitDirection = hitDirection;

    std::vector<double> qCur(qStart.data(), qStart.data() + qStart.rows() * qStart.cols());
    std::vector<double> grad(7);

    double opt_fun;
    auto result = optimizer.optimize(qCur, opt_fun);


    Eigen::Vector3d xCur;
    Kinematics::JointArrayType jointCur(qCur.data());
    return false;
}

double HittingPointOptimizer::objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data) {
    auto optData = (OptimizerData *) f_data;
    if (!grad.empty()){
        numerical_grad(f, x, optData, grad);
    }
    return f(x, optData);
}

double HittingPointOptimizer::equalityConstraint(const vector<double> &x, vector<double> &grad, void *f_data) {
    auto optData = (OptimizerData *) f_data;
    if (!grad.empty()){
        numerical_grad(h, x, optData, grad);
    }
    return h(x, optData);
}

double HittingPointOptimizer::f(const vector<double> &x, const OptimizerData* data) {
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
        grad[i] = (function(x_pos, data) - function(x_neg, data))/ (2 * data->epsilon);
    }
}


int main(int argc, char *argv[]) {
    iiwas_kinematics::Kinematics kinematics(Eigen::Vector3d(0, 0., 0.515),
                                            Eigen::Quaterniond(1., 0., 0., 0.));
    HittingPointOptimizer optimizer(kinematics);
    Eigen::Vector3d hitPos, hitDir;
    iiwas_kinematics::Kinematics::JointArrayType qStart, qOut;

    hitPos << 0.8, 0.2, 0.2;
    hitDir << 1.0, -0.2, 0.0;
    hitDir.normalize();

    qStart.setZero();

    cout << "#################################" << endl;
    cout << "#      Test Optimization        #" << endl;
    cout << "#################################" << endl;
    auto start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i) {
        optimizer.optimize(hitPos, hitDir, qStart, qOut);
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Optimization Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 100. / 1.e6 << "ms\n";

    return 0;
}


