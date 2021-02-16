//
// Created by puze on 16.02.21.
//
#include <iostream>
#include "air_hockey_baseline_agent/hitting_point_optimizer.h"
#include <chrono>

using namespace std;
using namespace nlopt;
using namespace iiwas_kinematics;

int main(int argc, char *argv[]) {
	iiwas_kinematics::Kinematics kinematics(Eigen::Vector3d(0, 0., 0.515),
	                                        Eigen::Quaterniond(1., 0., 0., 0.));
	HittingPointOptimizer optimizer(kinematics);
	Eigen::Vector3d hitPos, hitDir;
	iiwas_kinematics::Kinematics::JointArrayType qInOut;
	iiwas_kinematics::Kinematics::JacobianPosType jac;

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