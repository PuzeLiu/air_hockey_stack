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

#include "air_hockey_baseline_agent/system_state.h"

using namespace std;
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

SystemState::SystemState(const string& ns) {
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

	currentTactic = Tactics::INIT;

	isNewTactics = true;

	staticCount = 0;
	approachingCount = 0;
}

void SystemState::getPlannedJointState(Kinematics::JointArrayType &q,
                                       Kinematics::JointArrayType &dq, ros::Time &tStart, double offset_t) {
	q = observation.jointPosition;
	dq = observation.jointVelocity;

	if (!jointTrajectory.points.empty()) {
		tStart = ros::Time::now() + ros::Duration(offset_t);
		ros::Time tLast = jointTrajectory.header.stamp
		                  + jointTrajectory.points.back().time_from_start;
		ROS_INFO_STREAM("DEBUG##########" << tStart<< " ############ " << tLast);

		if (tStart <= tLast) {
			for (int i = 0; i < jointTrajectory.points.size(); ++i) {
				if (tStart < jointTrajectory.header.stamp + jointTrajectory.points[i].time_from_start) {
					for (int j = 0; j < NUM_OF_JOINTS; ++j) {
						q[j] = jointTrajectory.points[i].positions[j];
						dq[j] = jointTrajectory.points[i].velocities[j];
						tStart = jointTrajectory.header.stamp + jointTrajectory.points[i].time_from_start;
					}
					break;
				}
			}
		} else {
			for (int j = 0; j < NUM_OF_JOINTS; ++j) {
				q[j] = jointTrajectory.points.back().positions[j];
				dq[j] = jointTrajectory.points.back().velocities[j];
				tStart = jointTrajectory.header.stamp + jointTrajectory.points.back().time_from_start;
			}
		}
	}
}

bool SystemState::hasActiveTrajectory() {
	if (!jointTrajectory.points.empty()){
		return ros::Time::now() < jointTrajectory.header.stamp + jointTrajectory.points.back().time_from_start;
	} else {
		return false;
	}
}

bool SystemState::isPuckStatic() {
	return staticCount > 10;
}

bool SystemState::isPuckApproaching() {
	return approachingCount > 10;
}

void SystemState::updateObservationAndState(ObservationState observationState,
											const AgentParams& agentParams){
	observation = observationState;

	if (observation.puckPredictedState.state.block<2, 1>(2, 0).norm() < agentParams.vDefendMin){
		++staticCount;
	} else {
		staticCount = 0;
	}

	if (observationState.puckEstimatedState.dx() < 0){
		++approachingCount;
	} else {
		approachingCount = 0;
	}

	staticCount = min(staticCount, 20);
	approachingCount = min(approachingCount, 20);

}