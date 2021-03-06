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

#ifndef AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_TACTICS_TACTIC_H_
#define AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_TACTICS_TACTIC_H_

#include "air_hockey_baseline_agent/trajectory_generator.h"
#include <random>

namespace air_hockey_baseline_agent {

class Tactic {
public:
	Tactic(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	virtual ~Tactic();
	virtual bool ready() = 0;
	virtual bool apply() = 0;
	virtual void updateTactic();

protected:
	virtual void setNextState() = 0;
	bool planReturnTraj(const double &vMax,
			trajectory_msgs::MultiDOFJointTrajectory &cartTrajReturn,
			trajectory_msgs::JointTrajectory &jointTrajReturn);
	void setTactic(Tactics tactic);
	std::string tactic2String(Tactics tactic);
	void generateStopTrajectory();

protected:
	bool canSmash();
	bool shouldCut();
	bool shouldRepel();
	bool puckStuck();

protected:
	SystemState &state;
	TrajectoryGenerator &generator;
	AgentParams &agentParams;
	EnvironmentParams &envParams;

};

class Init: public Tactic {
public:
	Init(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~Init() override;
};

class Home: public Tactic {
public:
	Home(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~Home() override;
};

class Ready: public Tactic {
public:
	Ready(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~Ready() override;

protected:
	bool failed;
	int debugCount;
};

class Cut: public Tactic {
public:
	Cut(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~Cut() override;

protected:
	void calculateCutPosition();
	bool generateCutTrajectory(JointArrayType &qStart,
	                           JointArrayType &dqStart,
	                           ros::Time tStart);
	Eigen::Vector2d xCut, xCutPrev;
	int collisionNumPrev, waitForSteps, differenceCount;
};

class Repel: public Tactic {
public:
	Repel(EnvironmentParams &envParams, AgentParams &agentParams,
	    SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~Repel() override;
protected:
	bool generateRepelTrajectory(const JointArrayType &qCur, ros::Time &tStart);

	int counter;
};

class Prepare: public Tactic {
public:
	Prepare(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~Prepare() override;

protected:
	bool generatePrepareTrajectory(JointArrayType &qStart,
	                               JointArrayType &dqStart,
	                               ros::Time tStart);

	void getPreparePosAndVel(const Eigen::Vector2d &xStart,
						     Eigen::Vector2d &xPuck,
						     Eigen::Vector2d &xPrepare,
						     Eigen::Vector2d &vPrepare);
};

class MovePuck: public Tactic {
public:
	MovePuck(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~MovePuck() override;
};

class Smash: public Tactic {

public:
	Smash(EnvironmentParams &envParams, AgentParams &agentParams,
			SystemState &state, TrajectoryGenerator *generator);
	bool ready() override;
	bool apply() override;
	void setNextState() override;
	~Smash() override;

protected:
	Eigen::Vector3d computeTarget(Eigen::Vector3d puckPosition);
	Eigen::Vector2d getStopPoint(Eigen::Vector2d hitPoint);
    void getHitPointVelocity(Eigen::Vector2d &xHit2d, Eigen::Vector2d &vHit2d,
                             JointArrayType &qHitRef);
    bool generateHitTrajectory(const JointArrayType &qCur, ros::Time &tStart);

protected:
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_int_distribution<int> dist;
	Eigen::Vector2d xCur2d, xHit2d, vHit2d;
};

}

#endif /* AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_TACTICS_TACTIC_H_ */
