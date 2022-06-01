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

#ifndef AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_SYSTEM_STATE_H_
#define AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_SYSTEM_STATE_H_


#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "data_structures.h"
namespace air_hockey_baseline_agent {
	class SystemState {
	public:
		SystemState();

		void init(const AgentParams &agentParams);

		bool hasActiveTrajectory();

		bool isPuckStatic();

		bool isPuckApproaching();

		void updateObservationAndState(air_hockey_baseline_agent::ObservationState observationState,
		                               const air_hockey_baseline_agent::AgentParams &agentParams);

	public:
        TrajectoryBuffer trajectoryBuffer;

		air_hockey_baseline_agent::ObservationState observation;
		air_hockey_baseline_agent::Tactics currentTactic;

		double tNewTactics;
		bool isNewTactics;
		int staticCount;
		int approachingCount;

		JointArrayType qPlan, dqPlan;
        trajectory_msgs::JointTrajectoryPoint planPrevPoint;
		Eigen::Vector3d xPlan, vPlan;
		ros::Time tPlan, tStart;
	};
}

#endif //AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_SYSTEM_STATE_H_
