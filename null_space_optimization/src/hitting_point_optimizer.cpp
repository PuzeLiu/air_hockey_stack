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

    double tol = 1e-4;
    optimizer.add_equality_constraint(this->equalityConstraint, &optData, tol);

    std::vector<double> qLower(optData.kinematics.posLimitsLower_.data(),
                             optData.kinematics.posLimitsLower_.data() +
                             optData.kinematics.posLimitsLower_.cols() * optData.kinematics.posLimitsLower_.rows());
    optimizer.set_lower_bounds(qLower);

    std::vector<double> qUpper(optData.kinematics.posLimitsUpper_.data(),
                               optData.kinematics.posLimitsUpper_.data() +
                               optData.kinematics.posLimitsUpper_.cols() * optData.kinematics.posLimitsUpper_.rows());
    optimizer.set_upper_bounds(qUpper);


//    optimizer.set_ftol_rel(1e-8);
    optimizer.set_ftol_abs(1e-4);
    optimizer.set_xtol_rel(1e-8);
}

HittingPointOptimizer::~HittingPointOptimizer(){

}

bool HittingPointOptimizer::solve(const Eigen::Vector3d& hitPoint, const Eigen::Vector3d& hitDirection,
                                     iiwas_kinematics::Kinematics::JointArrayType& qInOut, double &velMagMax) {
    optData.hitPoint = hitPoint;
    optData.hitDirection = hitDirection;

    if (!getInitPoint(qInOut)){
        return false;
    }

    std::vector<double> qCur(qInOut.data(), qInOut.data() + qInOut.rows() * qInOut.cols());
    std::vector<double> grad(7);

    double opt_fun;
    auto result = optimizer.optimize(qCur, opt_fun);

    if (result < 0){
        return false;
    }

    if (h(qCur, &optData) < 1e-4) {
        for (int i = 0; i < qCur.size(); ++i) {
            qInOut[i] = qCur[i];
        }

        velMagMax = getMaxVelocity(qInOut);

        return true;
    } else {
        cout << "The position error is : " << h(qCur, &optData) << " bigger than 1e-4" << endl;
        cout << "termination condition:" << nlopt_result_to_string(nlopt_result(result)) << endl;
        return false;
    }
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

bool HittingPointOptimizer::getInitPoint(iiwas_kinematics::Kinematics::JointArrayType &qInOut) {
    iiwas_kinematics::Kinematics::JointArrayType qIk;
    qIk.setZero();
    if (!optData.kinematics.numericalInverseKinematics(optData.hitPoint, qIk, 1e-4, 200)){
        qInOut = qIk;
        cout << "No feasible IK solution" << endl;
        return false;
    }
    qInOut = qIk;
    return true;
}

double HittingPointOptimizer::getMaxVelocity(const iiwas_kinematics::Kinematics::JointArrayType &q) {
    Kinematics::JacobianPosType jac;
    optData.kinematics.jacobianPos(q, jac);
    auto jacInv = jac.transpose() * (jac * jac.transpose()).inverse();
    auto qRef = jacInv * optData.hitDirection;
    double min_scale = 10;
    for (int i = 0; i < q.size(); ++i) {
        min_scale = min(min_scale, abs(qRef[i]) / optData.kinematics.velLimitsUpper_[i]);
    }
    return optData.kinematics.velLimitsUpper_.cwiseProduct(qRef.cwiseAbs().cwiseInverse()).minCoeff();
}


int main(int argc, char *argv[]) {
    iiwas_kinematics::Kinematics kinematics(Eigen::Vector3d(0, 0., 0.515),
                                            Eigen::Quaterniond(1., 0., 0., 0.));
    HittingPointOptimizer optimizer(kinematics);
    Eigen::Vector3d hitPos, hitDir;
    iiwas_kinematics::Kinematics::JointArrayType qInOut;

    qInOut << 1.7207, 0.6541, -1.859, -1.422, 0.1313, 1.3779, -0.0653;
    double velMag = 0.;

    cout << "#################################" << endl;
    cout << "#      Test Optimization        #" << endl;
    cout << "#################################" << endl;
    auto start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i) {
        hitPos = hitPos.setRandom();
        hitPos.x() = hitPos.x() * 0.3 + 1.0;
        hitPos.y() = hitPos.y() * 0.45;
        hitPos.z() = 0.1505;
        hitDir.setRandom();
        hitDir.z() = 0.;
        hitDir.normalize();

        bool ret = optimizer.solve(hitPos, hitDir, qInOut, velMag);
        if (!ret){
            cout << "optimization fail" << endl;
        } else{
            cout << "Hitting Point: " << hitPos.transpose() << endl;
            cout << "Hitting Velocity: " << (hitDir * velMag).transpose() << " Magnitude: " << velMag << endl;
            cout << "Joint Position: " << qInOut.transpose() << endl;
        }

    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Optimization Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 100. / 1.e6 << "ms\n";

    return 0;
}


