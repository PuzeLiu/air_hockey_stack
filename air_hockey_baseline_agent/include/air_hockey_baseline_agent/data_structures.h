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

#include "iiwas_kinematics/iiwas_kinematics.h"
#include "air_hockey_puck_tracker/PuckTracker.hpp"
#include "air_hockey_referee/referee.h"

namespace air_hockey_baseline_agent {
enum Tactics {
	INIT,
	HOME,
	READY,    //!< go to home position
	PREPARE,  //!< adjust the puck's position when smash fails
	CUT,      //!< defend the incoming puck to opponent's court regardless of direction
	SMASH,    //!< hit the static or slow-moving puck
	N_TACTICS
};

struct EnvironmentParams {
	double puckRadius_;
	double puckHeight_;
	double malletRadius_;
	double tableLength_;
	double tableWidth_;
	double universalJointHeight_;
	double prepareHeight_;
	double initHeight_;

	Eigen::Vector3d tcp_position;
	Eigen::Quaterniond tcp_quaternion;
};


struct AgentParams {
	iiwas_kinematics::Kinematics::JointArrayType qRef_;
	iiwas_kinematics::Kinematics::JointArrayType qHome_;
	Eigen::Vector2d xHome_;
	Eigen::Vector2d xGoal_;
	Eigen::Vector2d xPrepare_;
	Eigen::Vector2d hitRange_;

	double vHitMax_;
	double vDefendMin_;
	double tDefendMin_;
	double defendLine_;
	double planTimeOffset_;
};

struct ObservationState {
	iiwas_kinematics::Kinematics::JointArrayType jointPosition;
	iiwas_kinematics::Kinematics::JointArrayType jointVelocity;
	iiwas_kinematics::Kinematics::JointArrayType jointDesiredPosition;
	iiwas_kinematics::Kinematics::JointArrayType jointDesiredVelocity;
    State puckEstimatedState;
    PuckPredictedState puckPredictedState;
    air_hockey_referee::GameStatus gameStatus;
};

struct AgentState {
	AgentState();

	Tactics currentTactic;
	GameStatus gameStatusPrev;
	ObservationState observation;
	double cutPrevY;
	int staticCount;
	int smashCount;
	int cutCount;

};

}



#endif /* AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_DATA_STRUCTURES_H_ */
