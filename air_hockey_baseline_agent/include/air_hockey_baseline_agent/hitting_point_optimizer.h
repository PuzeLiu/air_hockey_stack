//
// Created by puze on 11.02.21.
//

#ifndef SRC_HITTING_POINT_OPTIMIZER_H
#define SRC_HITTING_POINT_OPTIMIZER_H

#include <nlopt.hpp>
#include "iiwas_kinematics/iiwas_kinematics.h"

namespace air_hockey_baseline_agent {
    struct OptimizerData {
        iiwas_kinematics::Kinematics &kinematics;
        Eigen::Vector3d hitPoint;
        Eigen::Vector3d hitDirection;
        double epsilon;

        OptimizerData(iiwas_kinematics::Kinematics &kinematics);
    };

    class HittingPointOptimizer {
    public:
        HittingPointOptimizer(iiwas_kinematics::Kinematics &kinematics);

        ~HittingPointOptimizer();

        bool solve(const Eigen::Vector3d &hitPoint, const Eigen::Vector3d &hitDirection,
                   iiwas_kinematics::Kinematics::JointArrayType &qInOut, double &velMagMax);

    private:
        typedef double (*functype)(const std::vector<double> &x, const OptimizerData *data);

        static double objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data);

        static double equalityConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

        static double f(const std::vector<double> &x, const OptimizerData *data);

        static double h(const std::vector<double> &x, const OptimizerData *data);

        static void numerical_grad(functype function, const std::vector<double> &x, const OptimizerData *data,
                                   std::vector<double> &grad);

        bool getInitPoint(iiwas_kinematics::Kinematics::JointArrayType &qInOut);

        double getMaxVelocity(const iiwas_kinematics::Kinematics::JointArrayType &q);


    private:
        nlopt::opt optimizer;

        OptimizerData optData;

    };

};
#endif //SRC_HITTING_POINT_OPTIMIZER_H
