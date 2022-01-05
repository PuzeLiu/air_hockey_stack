#include <iostream>
#include <chrono>
#include <Eigen/Dense>

#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

class ProMP {
public:
    ProMP(Eigen::Vector2d bound_lower, Eigen::Vector2d bound_upper, double step, double height);

    ~ProMP();

    bool plan(const Eigen::Vector2d &x_start, const Eigen::Vector2d &v_start,
              const Eigen::Vector2d &x_hit, const Eigen::Vector2d &v_hit,
              trajectory_msgs::MultiDOFJointTrajectory &cartTraj);

    bool getParameters();

    Eigen::MatrixXd Phi(double t, int order = 0);

    double phase(double t);

    double dPhase(double t);

    Eigen::VectorXd basis(double z);

    Eigen::VectorXd dBasis(double z);

    Eigen::VectorXd ddBasis(double z);

    bool checkFeasibility(Eigen::Vector3d q_x, Eigen::Vector3d);

    Eigen::VectorXd center;
    Eigen::MatrixXd weight;
    Eigen::Vector2d boundLower, boundUpper;
    double width, tTotal, step, height;
    int nDim, nBasis;
    trajectory_msgs::MultiDOFJointTrajectoryPoint viaPoint;
};

using namespace Eigen;

ProMP::ProMP(Eigen::Vector2d bound_lower, Eigen::Vector2d bound_upper, double _step, double plane_height) {
    nDim = 2;

    step = _step;
    boundLower = bound_lower;
    boundUpper = bound_upper;
    height = plane_height;

    viaPoint.transforms.resize(1);
    viaPoint.velocities.resize(1);
    viaPoint.accelerations.resize(1);
}

ProMP::~ProMP() {}

bool ProMP::plan(const Eigen::Vector2d &x_start, const Eigen::Vector2d &v_start, const Eigen::Vector2d &x_hit,
                 const Eigen::Vector2d &v_hit, trajectory_msgs::MultiDOFJointTrajectory &cartTraj) {
    if (not getParameters()){
        return false;
    }

    cartTraj.points.clear();

    double tCurrent = 0.;

    while (tCurrent <= tTotal){

        MatrixXd Phi_t = Phi(tCurrent, 2);


        VectorXd q_x = Phi_t * weight.row(0).transpose();
        VectorXd q_y = Phi_t * weight.row(1).transpose();

//        if (!checkFeasibility(q_x, q_y)){
//            return false;
//        }

        viaPoint.transforms[0].translation.x = q_x[0];
        viaPoint.transforms[0].translation.y = q_y[0];
        viaPoint.transforms[0].translation.z = height;
        viaPoint.velocities[0].linear.x = q_x[1];
        viaPoint.velocities[0].linear.y = q_y[1];
        viaPoint.velocities[0].linear.z = 0.;
        viaPoint.accelerations[0].linear.x = q_x[2];
        viaPoint.accelerations[0].linear.y = q_y[2];
        viaPoint.accelerations[0].linear.z = 0.;
        viaPoint.time_from_start = ros::Duration(tCurrent);

        cartTraj.points.push_back(viaPoint);
        tCurrent += step;
        std::cout << (tCurrent <= tTotal) << std::endl;
    }

//    cartTraj.header.stamp = ros::Time::now();
    return true;
}

bool ProMP::getParameters() {
    nBasis = 20;
    width = 0.03;
    center = VectorXd::LinSpaced(nBasis, 0 - sqrt(width), 1 + sqrt(width));

    weight.resize(nDim, nBasis);
    weight.row(0) = VectorXd::LinSpaced(20, 1, 2);
    weight.row(1) = VectorXd::LinSpaced(20, 2, 3);
    std::cout << weight<< std::endl;
    tTotal = 5.;

    return true;
}

MatrixXd ProMP::Phi(double t, int order) {
    MatrixXd Phi(order + 1, nBasis);

    double z = phase(t);

    VectorXd b = basis(z);
    double B = b.sum();
    Phi.row(0) = b / B;
    if (order == 0) return Phi;

    VectorXd dbdz = dBasis(z);
    Phi.row(1) = (dbdz / B - b * pow(B, -2) * dbdz.sum()) * dPhase(t);
    if (order == 1) return Phi;

    VectorXd ddbdz = ddBasis(z);
    Phi.row(2) = (ddbdz / B - 2 * dbdz * pow(B, -2) * dbdz.sum() +
                  2 * b * pow(B, -3) * pow(dbdz.sum(), 2) - b * pow(B, -2) * ddbdz.sum()) * pow(dPhase(t), 2);
    if (order == 2) { return Phi; }
    else { throw std::logic_error("the order should be 1 or 2"); }
}

double ProMP::phase(double t) {
    return t / tTotal;
}

double ProMP::dPhase(double t) {
    return 1 / tTotal;
}

VectorXd ProMP::basis(double z) {
    return exp(-pow(z - center.array(), 2) / 2 / width);
}

VectorXd ProMP::dBasis(double z) {
    return exp(-pow(z - center.array(), 2) / 2 / width).cwiseProduct(-(z - center.array()) / width);
}

VectorXd ProMP::ddBasis(double z) {
    VectorXd z_Center = z - center.array();
    VectorXd exp_term = (-z_Center.array().square() / 2 / width).exp();
    return exp_term.array() * (z_Center.array() / width).square() - exp_term.array() / width;
}

bool ProMP::checkFeasibility(Eigen::Vector3d q_x, Eigen::Vector3d q_y) {
    if (q_x[0] > boundUpper[0] or q_y[0] > boundUpper[1] or
        q_x[0] < boundLower[0] or q_y[0] < boundLower[1]){
        return false;
    }
    return true;
}


int main() {
    double step = 0.01;
    Vector2d boundLower, boundUpper;
    boundLower << 0.05, -0.5;
    boundUpper << 0.8, 0.5;

    ProMP promp(boundLower, boundUpper, step, 0.3);

    trajectory_msgs::MultiDOFJointTrajectory cartTraj;
    Vector2d xStart, vStart, xHit, vHit;
    promp.plan(xStart, vStart, xHit, vHit, cartTraj);

    for (int i = 0; i < cartTraj.points.size(); ++i) {
        std::cout << "Point: "  << cartTraj.points[i].transforms[0].translation.x << " "
                                << cartTraj.points[i].transforms[0].translation.y << " "
                                << cartTraj.points[i].transforms[0].translation.z << std::endl
                << "Velocity: " << cartTraj.points[i].velocities[0].linear.x << " "
                                << cartTraj.points[i].velocities[0].linear.y << " "
                                << cartTraj.points[i].velocities[0].linear.z << std::endl
            << "Acceleration: " << cartTraj.points[i].accelerations[0].linear.x << " "
                                << cartTraj.points[i].accelerations[0].linear.y << " "
                                << cartTraj.points[i].accelerations[0].linear.z << std::endl;
    }
}
