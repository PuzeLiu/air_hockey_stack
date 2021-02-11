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

#include "air_hockey_baseline_agent/tactics.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;
using namespace Eigen;
using namespace iiwas_kinematics;
using namespace air_hockey_baseline_agent;

SystemState::SystemState(string ns_prefix) {
	cartTrajectory_.joint_names.push_back("x");
	cartTrajectory_.joint_names.push_back("y");
	cartTrajectory_.joint_names.push_back("z");

	for (int i = 1; i < 8; i++) {
		jointTrajectory_.joint_names.push_back(
				ns_prefix + "_joint_" + to_string(i));

	}

	restart = false;
}

void SystemState::getPlannedJointState(Kinematics::JointArrayType &q,
		Kinematics::JointArrayType &dq, ros::Time &tStart, double offset_t) {
	if (jointTrajectory_.points.size() > 0) {
		tStart = ros::Time::now() + ros::Duration(offset_t);
		ros::Time tLast = jointTrajectory_.header.stamp
				+ jointTrajectory_.points.back().time_from_start;
		if (tStart <= tLast) {
			for (int i = jointTrajectory_.points.size() - 1; i >= 0; --i) {
				if (tStart
						> jointTrajectory_.header.stamp
								+ jointTrajectory_.points[i].time_from_start) {
					for (int j = 0; j < NUM_OF_JOINTS; ++j) {
						q[j] = jointTrajectory_.points[i + 1].positions[j];
						dq[j] = jointTrajectory_.points[i + 1].velocities[j];
					}
					break;
				}
			}
		} else {
			for (int j = 0; j < NUM_OF_JOINTS; ++j) {
				q[j] = jointTrajectory_.points.back().positions[j];
				dq[j] = jointTrajectory_.points.back().velocities[j];
			}
		}
	}
}

void SystemState::getPlannedCartesianState(Vector3d &x, Vector3d &dx,
		Kinematics::JointArrayType &q, Kinematics::JointArrayType &dq,
		ros::Time &tStart, double t = 0) {
//	kinematics_->forwardKinematics(q, x);
//	Kinematics::JacobianPosType jacobian;
//	kinematics_->jacobianPos(q, jacobian);
//	dx = jacobian * dq;
}

bool Tactic::planReturnTraj(SystemState &state, const double &vMax,
		trajectory_msgs::MultiDOFJointTrajectory &cartTrajReturn,
		trajectory_msgs::JointTrajectory &jointTrajReturn) {
	double vReadyMax = vMax;
	cartTrajReturn.joint_names.push_back("x");
	cartTrajReturn.joint_names.push_back("y");
	cartTrajReturn.joint_names.push_back("z");

	trajectory_msgs::MultiDOFJointTrajectoryPoint lastPoint =
			state.cartTrajectory_.points.back();

	for (size_t i = 0; i < 10; ++i) {
		cartTrajReturn.points.clear();
		jointTrajReturn.points.clear();
		cartTrajReturn.points.push_back(lastPoint);

		Vector3d xStop;
		xStop << lastPoint.transforms[0].translation.x, lastPoint.transforms[0].translation.y, lastPoint.transforms[0].translation.z;
		generator.transformations->applyInverseTransform(xStop);
		Vector2d xStop2d = xStop.block<2, 1>(0, 0);
		double tStop = (xHome_ - xStop2d).norm() / vReadyMax;
		generator.cubicLinearMotion->plan(xStop2d, Vector2d(0., 0.), xHome_,
				Vector2d(0., 0.), tStop, cartTrajReturn);
		cartTrajReturn.points.erase(cartTrajReturn.points.begin());
		generator.transformations->transformTrajectory(cartTrajReturn);

		Kinematics::JointArrayType qStart;
		for (int j = 0; j < NUM_OF_JOINTS; ++j) {
			qStart[j] = state.jointTrajectory_.points.back().positions[j];
		}

		bool ok = generator.optimizer->optimizeJointTrajectoryAnchor(
				cartTrajReturn, qStart, qHome_, jointTrajReturn);

		if (ok) {
			return true;
		} else {
			vReadyMax *= 0.8;
			ROS_INFO_STREAM(
					"Optimization Failed [RETURN]. Reduce the velocity for Ready: " << vReadyMax);
		}
	}

	return false;
}

Tactic::Tactic(EnvironmentParams &envParams, AgentParams &agentParams,
		SystemState *state, TrajectoryGenerator *generator) :
		envParams(envParams), agentParams(agentParams), state(*state), generator(
				*generator) {

}

Tactic::~Tactic() {

}

