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

#include "air_hockey_baseline_agent/data_structures.h"

using namespace std;
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

SystemState::SystemState(string ns) {
	string ns_prefix;

	if (ns == "/iiwa_front") {
		ns_prefix = 'F';
	} else if (ns == "/iiwa_back") {
		ns_prefix = 'B';
	} else {
		exit(-1);
	}

	cartTrajectory.joint_names.push_back("x");
	cartTrajectory.joint_names.push_back("y");
	cartTrajectory.joint_names.push_back("z");

	for (int i = 1; i < 8; i++) {
		jointTrajectory.joint_names.push_back(
				ns_prefix + "_joint_" + to_string(i));

	}

	restart = false;
}

void SystemState::getPlannedJointState(Kinematics::JointArrayType &q,
		Kinematics::JointArrayType &dq, ros::Time &tStart, double offset_t) {
	if (jointTrajectory.points.size() > 0) {
		tStart = ros::Time::now() + ros::Duration(offset_t);
		ros::Time tLast = jointTrajectory.header.stamp
				+ jointTrajectory.points.back().time_from_start;
		if (tStart <= tLast) {
			for (int i = jointTrajectory.points.size() - 1; i >= 0; --i) {
				if (tStart
						> jointTrajectory.header.stamp
								+ jointTrajectory.points[i].time_from_start) {
					for (int j = 0; j < NUM_OF_JOINTS; ++j) {
						q[j] = jointTrajectory.points[i + 1].positions[j];
						dq[j] = jointTrajectory.points[i + 1].velocities[j];
					}
					break;
				}
			}
		} else {
			for (int j = 0; j < NUM_OF_JOINTS; ++j) {
				q[j] = jointTrajectory.points.back().positions[j];
				dq[j] = jointTrajectory.points.back().velocities[j];
			}
		}
	}
}

