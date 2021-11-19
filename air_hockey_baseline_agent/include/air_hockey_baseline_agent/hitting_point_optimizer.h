/*
 * MIT License
 * Copyright (c) 2020-2021 Puze Liu, Davide Tateo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

	    static double inEqualityConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

        static double f(const std::vector<double> &x, const OptimizerData *data);

        static double h(const std::vector<double> &x, const OptimizerData *data);

	    static double g(const std::vector<double> &x, const OptimizerData *data);

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
