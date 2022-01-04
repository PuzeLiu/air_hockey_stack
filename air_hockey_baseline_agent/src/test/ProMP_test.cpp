#include <iostream>
#include <chrono>
#include <Eigen/Dense>

class ProMP {
public:
    ProMP(int n_dim, int n_basis, Eigen::VectorXd center, double width, double t_total, double step);

    ~ProMP();

    Eigen::MatrixXd Phi(double t, int order = 0);

    double phase(double t);

    double dPhase(double t);

    Eigen::VectorXd basis(double z);

    Eigen::VectorXd dBasis(double z);

    Eigen::VectorXd ddBasis(double z);

private:
    Eigen::VectorXd center;
    double width, tTotal, step;
    int nDim, nBasis;
};

using namespace Eigen;

ProMP::ProMP(int _n_dim, int _n_basis, VectorXd _centers, double _width, double _t_total, double _step) {
    nDim = _n_dim;
    nBasis = _n_basis;
    center = _centers;
    width = _width;
    tTotal = _t_total;
    step = _step;
}

ProMP::~ProMP() {}

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


int main() {
    int nDim = 2;
    int nBasis = 20;
    double width = 0.03;
    double step = 0.01;
    VectorXd centers = VectorXd::LinSpaced(nBasis, 0 - sqrt(width), 1 + sqrt(width));

    double tTotal = 5;
    ProMP promp(nDim, nBasis, centers, width, tTotal, step);
    std::cout << promp.Phi(0.5, 2) << std::endl;
}
