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

#ifndef AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_DATA_STRUCTURES_H_
#define AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_DATA_STRUCTURES_H_

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "air_hockey_puck_tracker/PuckTracker.hpp"
#include "air_hockey_referee/referee.h"

namespace air_hockey_baseline_agent {
	typedef Eigen::VectorXd JointArrayType;

	enum Tactics {
		INIT = 0,      //!< go to init position
		HOME,      //!< go to home position from init
		READY,     //!< go to home position
		PREPARE,   //!< adjust the puck's position when smash fails
		CUT,       //!< defend the incoming puck and control it in our court
		REPEL,     //!< hit back the puck regardless of the direction
		SMASH,     //!< hit the static or slow-moving puck
		N_TACTICS  //!< Total number of available tacticsProcessor
	};

	struct EnvironmentParams {
		double puckRadius;
		double puckHeight;
		double malletRadius;
		double tableLength;
		double tableWidth;
	};

	struct AgentParams {
		std::string name;
        bool debugTactics;

		pinocchio::Model pinoModel;
		pinocchio::Data pinoData;
		int pinoFrameId;
		int nq;
		int smashStrategy;

		JointArrayType qRef;
		JointArrayType qHome;
		JointArrayType qInit;
		Eigen::Vector2d xGoal;
		Eigen::Vector3d xHome;
		Eigen::Vector3d xPrepare;

		Eigen::Vector2d hitRange;

		double vDefendMin;
		double tDefendMin;
		double defendZoneWidth;
		double defendLine;
		double planTimeOffset;
		double tPredictionMax;
		double tTacticSwitchMin;
		double hitVelocityScale;
		double initHeight;
		double universalJointHeight;
	};

	struct ObservationState {
		ros::Time stamp;
		JointArrayType jointPosition;
		JointArrayType jointVelocity;
		JointArrayType jointDesiredPosition;
		JointArrayType jointDesiredVelocity;
		PuckState puckEstimatedState;
		PuckPredictedState puckPredictedState;
		air_hockey_referee::GameStatus gameStatus;
	};

	struct OptimizerData {
		// Data for planner
		Eigen::Vector3d hitPoint;
		Eigen::Vector3d hitDirection;
		double epsilon;

		// Data for Null Space Optimizer
		int dimNullSpace;

		// Variable for QP Solver
		double rate;
		Eigen::SparseMatrix<double>P;
		Eigen::VectorXd q;
		Eigen::SparseMatrix<double> A;
		Eigen::VectorXd K;            // Weight for correcting position error
		Eigen::Vector3d xCurPos;
		Eigen::MatrixXd N_J;
		Eigen::MatrixXd J;
		JointArrayType weights, weightsAnchor;
		JointArrayType upperBound, lowerBound;
		Eigen::VectorXd alphaLast;

		AgentParams& agentParams;

		OptimizerData(AgentParams& params): agentParams(params){
			epsilon = sqrt(std::numeric_limits<double>::epsilon());
			dimNullSpace = 4;

			J.resize(3, agentParams.pinoModel.nv);
			N_J.resize(agentParams.pinoModel.nv, agentParams.pinoModel.nv-3);
			weights.resize(agentParams.nq);
			weightsAnchor.resize(agentParams.nq);
			upperBound.resize(agentParams.nq);
			lowerBound.resize(agentParams.nq);

			q.resize(dimNullSpace);

			P.resize(dimNullSpace, dimNullSpace);
			q.resize(dimNullSpace);
			A.resize(agentParams.pinoModel.nq, dimNullSpace);
			P.setIdentity();
			A.setZero();
			K.resize(3);
			alphaLast.resize(dimNullSpace);
			alphaLast.setZero();
		}
	};
}


#endif /* AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_DATA_STRUCTURES_H_ */
