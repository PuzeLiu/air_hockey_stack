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

#ifndef AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_TACTICS_TACTIC_H_
#define AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_TACTICS_TACTIC_H_

#include "air_hockey_baseline_agent/trajectory_generator.h"
#include <random>

namespace air_hockey_baseline_agent {

class SystemState {
public:
	SystemState(std::string ns_prefix);

	void getPlannedJointState(iiwas_kinematics::Kinematics::JointArrayType &q,
			iiwas_kinematics::Kinematics::JointArrayType &dq, ros::Time &tStart,
			double offset_t);
	void getPlannedCartesianState(Eigen::Vector3d &x, Eigen::Vector3d &dx,
			iiwas_kinematics::Kinematics::JointArrayType &q,
			iiwas_kinematics::Kinematics::JointArrayType &dq, ros::Time &tStart,
			double t = 0);

public:
	trajectory_msgs::MultiDOFJointTrajectory cartTrajectory_;
	trajectory_msgs::JointTrajectory jointTrajectory_;
	ros::Time trajStopTime_;

	bool restart;

};

class Tactic {
public:
	Tactic(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready() = 0;
	virtual bool apply() = 0;
	virtual ~Tactic();

protected:
	bool planReturnTraj(const double &vMax,
			trajectory_msgs::MultiDOFJointTrajectory &cartTrajReturn,
			trajectory_msgs::JointTrajectory &jointTrajReturn);

protected:
	SystemState &state;
	TrajectoryGenerator &generator;
	AgentParams &agentParams;
	EnvironmentParams &envParams;

};

class Init: public Tactic {
public:
	Init(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready();
	virtual bool apply();
	virtual ~Init();

private:
	iiwas_kinematics::Kinematics::JointArrayType qInit;

	void computeQInit();
};

class Home: public Tactic {
public:
	Home(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready();
	virtual bool apply();
	virtual ~Home();

private:
	iiwas_kinematics::Kinematics::JointArrayType qHome;
};

class Ready: public Tactic {
public:
	Ready(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready();
	virtual bool apply();
	virtual ~Ready();
};

class Cut: public Tactic {
public:
	Cut(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready();
	virtual bool apply();
	virtual ~Cut();
};

class Prepare: public Tactic {
public:
	Prepare(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready();
	virtual bool apply();
	virtual ~Prepare();
};

class MovePuck: public Tactic {
public:
	MovePuck(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready();
	virtual bool apply();
	virtual ~MovePuck();
};

class Smash: public Tactic {
public:
	Smash(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState *state, TrajectoryGenerator *generator);
	virtual bool ready();
	virtual bool apply();
	virtual ~Smash();

private:
	Eigen::Vector2d computeTarget(Eigen::Vector2d puckPosition);

private:
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_int_distribution<int> dist;

};

}

#endif /* AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_TACTICS_TACTIC_H_ */
