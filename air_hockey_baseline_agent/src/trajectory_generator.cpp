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

#include "air_hockey_baseline_agent/trajectory_generator.h"

using namespace Eigen;
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

TrajectoryGenerator::TrajectoryGenerator(std::string ns, EnvironmentParams data, double rate) {
	kinematics = new Kinematics(data.tcp_position, data.tcp_quaternion);
	optimizer = new NullSpaceOptimizer(kinematics);
	transformations = new Transformations(ns);
	hittingPointOptimizer = new HittingPointOptimizer(*kinematics);

	Vector2d bound_lower(data.malletRadius + 0.02,
			-data.tableWidth / 2 + data.malletRadius + 0.02);
	Vector2d bound_upper(data.tableLength / 2 - data.malletRadius,
			data.tableWidth / 2 - data.malletRadius - 0.02);

	combinatorialHit = new CombinatorialHit(bound_lower, bound_upper, rate,
			data.universalJointHeight);
	cubicLinearMotion = new CubicLinearMotion(rate, data.universalJointHeight);

}

void TrajectoryGenerator::getCartesianPosAndVel(Vector3d &x, Vector3d &dx,
		Kinematics::JointArrayType &q, Kinematics::JointArrayType &dq) {
	kinematics->forwardKinematics(q, x);
	Kinematics::JacobianPosType jacobian;
	kinematics->jacobianPos(q, jacobian);
	dx = jacobian * dq;
}

TrajectoryGenerator::~TrajectoryGenerator() {
	delete kinematics;
	delete optimizer;
	delete transformations;
	delete combinatorialHit;
	delete cubicLinearMotion;
}

