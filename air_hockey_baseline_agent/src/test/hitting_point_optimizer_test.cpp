//
// Created by puze on 16.02.21.
//
#include <iostream>
#include <fstream>
#include "air_hockey_baseline_agent/hitting_point_optimizer.h"
#include <chrono>
#include <ros/package.h>

using namespace std;
using namespace nlopt;
using namespace air_hockey_baseline_agent;

void loadAgentParams(AgentParams& agentParams){
	agentParams.name = "/iiwa_front";

	string parent_dir = ros::package::getPath("air_hockey_description");
	string robot_description = parent_dir + "/urdf/iiwa_striker_extension_only.urdf";
	pinocchio::urdf::buildModel(robot_description, agentParams.pinoModel);
	agentParams.pinoData = pinocchio::Data(agentParams.pinoModel);
	agentParams.pinoFrameId = agentParams.pinoModel.getFrameId("F_striker_tip");
	agentParams.nq = agentParams.pinoModel.nq;

	agentParams.qRef.resize(agentParams.pinoModel.nq);
	agentParams.qRef << 0., 0.15378149,  0., -1.3630843, 0.,  1.3767066, 0.;
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