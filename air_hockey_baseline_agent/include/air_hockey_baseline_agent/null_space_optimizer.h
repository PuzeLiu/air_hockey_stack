/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
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

#ifndef SRC_NULL_SPACE_OPTIMIZER_H
#define SRC_NULL_SPACE_OPTIMIZER_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <angles/angles.h>

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>

#include "observer.h"
#include "data_structures.h"

namespace air_hockey_baseline_agent {
    class NullSpaceOptimizer {
    public:
        NullSpaceOptimizer(AgentParams& agentParams, OptimizerData& optimizationData);

        ~NullSpaceOptimizer();

        bool optimizeJointTrajectory(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                     const JointArrayType &qStart,
                                     trajectory_msgs::JointTrajectory &jointTraj);

        bool optimizeJointTrajectoryAnchor(const trajectory_msgs::MultiDOFJointTrajectory &cartTraj,
                                           const JointArrayType &qStart,
                                           const JointArrayType &qAnchor,
                                           trajectory_msgs::JointTrajectory &jointTraj,
                                           bool increasing=true);

        void SolveJoint7(JointArrayType &q, JointArrayType &dq);

    private:
        bool solveQP(const Eigen::Vector3d& xDes,
                     const Eigen::Vector3d& dxDes,
                     const JointArrayType& qCur,
					 JointArrayType& qNext,
					 JointArrayType& dqNext);

        bool solveQPAnchor(const Eigen::Vector3d &xDes,
                           const Eigen::Vector3d &dxDes,
                           const JointArrayType &qCur,
                           const JointArrayType &qAnchor,
						   JointArrayType &qNext,
						   JointArrayType &dqNext);


    private:
		AgentParams& agentParams;
	    OptimizerData& optData;
        OsqpEigen::Solver solver;
    };
}


#endif //SRC_NULL_SPACE_OPTIMIZER_H
