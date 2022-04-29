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

#include "air_hockey_baseline_agent/hitting_point_optimizer.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <ros/package.h>

using namespace std;
using namespace nlopt;
using namespace air_hockey_baseline_agent;

void loadAgentParams(AgentParams& agentParams){
	agentParams.name = "/iiwa_front";
	agentParams.debugTactics = false;

	string parent_dir = ros::package::getPath("air_hockey_description");
	string robot_description = parent_dir + "/urdf/iiwa_striker.urdf";
	pinocchio::urdf::buildModel(robot_description, agentParams.pinoModel);
	agentParams.pinoData = pinocchio::Data(agentParams.pinoModel);
	agentParams.pinoFrameId = agentParams.pinoModel.getFrameId("F_striker_tip");
	agentParams.nq = agentParams.pinoModel.nq;

	agentParams.qRef.resize(agentParams.pinoModel.nq);
	agentParams.qRef << 0., 0.06580,  0., -1.45996, 0.,  1.22487, 0.;
	agentParams.xGoal << 1.98, 0.0;
	agentParams.xHome << 0.08, 0.0, 0.0;
	agentParams.xPrepare << 0.4, 0.0, 0.0;

	agentParams.hitRange << 0.2, 0.9;

	agentParams.vDefendMin = 0.08;
	agentParams.tDefendMin = 0.3;
	agentParams.defendZoneWidth = 0.4;
	agentParams.defendLine = 0.2;
	agentParams.planTimeOffset = 0.1;
	agentParams.tPredictionMax = 1.5;
	agentParams.tTacticSwitchMin = 1.5;
	agentParams.hitVelocityScale = 1.0;
	agentParams.initHeight = 0.2;
}

int main(int argc, char *argv[]) {
	AgentParams agentParams;
	loadAgentParams(agentParams);
	OptimizerData optData(agentParams);
	HittingPointOptimizer optimizer(agentParams, optData);
	Eigen::Vector3d hitPos, hitDir;
	JointArrayType qInOut(agentParams.nq);

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
			cout << "Hitting Point: " << hitPos.transpose() << " Magnitude: " << velMag << endl;
		}
	}
	auto finish = chrono::high_resolution_clock::now();
	cout << "Optimization Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 100. / 1.e6 << "ms\n";

	return 0;
}