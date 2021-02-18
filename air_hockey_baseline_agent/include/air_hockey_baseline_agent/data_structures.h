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
#ifndef AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_DATA_STRUCTURES_H_
#define AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_DATA_STRUCTURES_H_

#include <Eigen/Dense>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "iiwas_kinematics/iiwas_kinematics.h"
#include "air_hockey_puck_tracker/PuckTracker.hpp"
#include "air_hockey_referee/referee.h"

namespace air_hockey_baseline_agent {
enum Tactics {
	INIT = 0,      //!< go to init position
	HOME,      //!< go to home position from init
	READY,     //!< go to home position
	PREPARE,   //!< adjust the puck's position when smash fails
	CUT,       //!< defend the incoming puck to opponent's court regardless of direction
	SMASH,     //!< hit the static or slow-moving puck
	N_TACTICS  //!< Total number of available tacticsProcessor
};

struct EnvironmentParams {
	double puckRadius;
	double puckHeight;
	double malletRadius;
	double tableLength;
	double tableWidth;
	double universalJointHeight;
	double prepareHeight;
	double initHeight;

	Eigen::Vector3d tcp_position;
	Eigen::Quaterniond tcp_quaternion;
};


struct AgentParams {
	iiwas_kinematics::Kinematics::JointArrayType qRef;
	iiwas_kinematics::Kinematics::JointArrayType qHome;
	iiwas_kinematics::Kinematics::JointArrayType qInit;
	Eigen::Vector2d xGoal;
	Eigen::Vector3d xHome;
	Eigen::Vector3d xPrepare;

	Eigen::Vector2d hitRange;

	double vDefendMin;
	double tDefendMin;
	double defendWidth;
	double defendLine;
	double planTimeOffset;
	double tPredictionMax;
	double tTacticSwitchMin;
};

struct ObservationState {
	iiwas_kinematics::Kinematics::JointArrayType jointPosition;
	iiwas_kinematics::Kinematics::JointArrayType jointVelocity;
	iiwas_kinematics::Kinematics::JointArrayType jointDesiredPosition;
	iiwas_kinematics::Kinematics::JointArrayType jointDesiredVelocity;
    PuckState puckEstimatedState;
    PuckPredictedState puckPredictedState;
    air_hockey_referee::GameStatus gameStatus;
};

}



#endif /* AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_DATA_STRUCTURES_H_ */
