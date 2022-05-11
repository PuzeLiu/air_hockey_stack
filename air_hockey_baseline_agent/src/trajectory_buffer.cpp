/*
 * MIT License
 * Copyright (c) 2020-2022 Puze Liu, Davide Tateo
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


#include "air_hockey_baseline_agent/data_structures.h"

using namespace std;
using namespace air_hockey_baseline_agent;

TrajectoryBuffer::TrajectoryBuffer(){};

TrajectoryBuffer::TrajectoryBuffer(const AgentParams& agentParams){
    trajectoryBuffer.resize(2);

    for (int i = 0; i < 2; ++i)
    {
        trajectoryBuffer[i].cartTrajectory.joint_names.push_back("x");
        trajectoryBuffer[i].cartTrajectory.joint_names.push_back("y");
        trajectoryBuffer[i].cartTrajectory.joint_names.push_back("z");

        for (int j = 0; j < agentParams.nq; ++j)
        {
            trajectoryBuffer[i].jointTrajectory.joint_names.push_back(agentParams.pinoModel.names[j + 1]);
        }

        trajectoryBuffer[i].cartTrajectory.points.push_back(trajectory_msgs::MultiDOFJointTrajectoryPoint());
        trajectoryBuffer[i].jointTrajectory.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    }

    execIdx = 0;
};
