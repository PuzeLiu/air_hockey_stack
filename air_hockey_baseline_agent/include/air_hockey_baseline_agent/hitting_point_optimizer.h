//
// Created by puze on 11.02.21.
//

#ifndef SRC_HITTING_POINT_OPTIMIZER_H
#define SRC_HITTING_POINT_OPTIMIZER_H

#include <nlopt.hpp>
#include <coin/ClpSimplex.hpp>
#include <coin/ClpSolve.hpp>
#include "utils.h"
#include "data_structures.h"

namespace air_hockey_baseline_agent {
    class HittingPointOptimizer {
    public:
        HittingPointOptimizer(AgentParams& agentParams, OptimizerData& optData);

        ~HittingPointOptimizer();

        bool solve(const Eigen::Vector3d &hitPoint, const Eigen::Vector3d &hitDirection,
                   JointArrayType &qInOut, double &velMagMax);

    private:
        typedef double (*functype)(const std::vector<double> &x, const OptimizerData *data);

        static double objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data);

        static double equalityConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

        static double f(const std::vector<double> &x, const OptimizerData *data);

        static double h(const std::vector<double> &x, const OptimizerData *data);

        static void numerical_grad(functype function, const std::vector<double> &x, const OptimizerData *data,
								   std::vector<double> &grad);

        bool getInitPoint(JointArrayType &qInOut);

        double getMaxVelocity(const JointArrayType &q);

        double getMaxVelocityLP(const JointArrayType &q);


    private:
        nlopt::opt nlSolver;

		ClpSimplex simplexModel;

		AgentParams& agentParams;
        OptimizerData& optData;
    };

};
#endif //SRC_HITTING_POINT_OPTIMIZER_H
