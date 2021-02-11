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

#ifndef SRC_TACTICAL_AGENT_H
#define SRC_TACTICAL_AGENT_H

#include "air_hockey_baseline_agent/data_structures.h"
#include "air_hockey_baseline_agent/tactics.h"
#include "air_hockey_baseline_agent/trajectory_generator.h"
#include "air_hockey_baseline_agent/observer.h"
#include "air_hockey_baseline_agent/null_space_optimizer.h"
#include "air_hockey_baseline_agent/transformations.h"

using namespace std;

namespace air_hockey_baseline_agent {

class Agent {
public:
	Agent(ros::NodeHandle nh, double rate);
	~Agent();

	void start();

	void update();

private:
	void loadAgentParam();
	void updateTactic();
	void publishTrajectory();

	double updateGoal(Eigen::Vector2d puckPos);
	bool setTactic(Tactics tactic);
	void loadEnvironmentParams();
	std::string getControllerName();
	void loadTactics();

private:
	// ROS
	ros::NodeHandle nh;
	ros::Publisher jointTrajectoryPub;
	ros::Publisher cartTrajectoryPub;
	ros::Rate rate;

	// PARAMS
	EnvironmentParams envParams;
	AgentParams agentParams;

	// LOGIC
	Observer *observer;
	TrajectoryGenerator *generator;
	std::vector<Tactic*> tactics;

	//State
	SystemState state;
	AgentState agentState;
};

}

#endif //SRC_TACTICAL_AGENT_H
