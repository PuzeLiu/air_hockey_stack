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

#ifndef AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_TRAJECTORY_GENERATOR_H_
#define AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_TRAJECTORY_GENERATOR_H_

#include "air_hockey_baseline_agent/data_structures.h"

//#include "air_hockey_baseline_agent/planner/bezier_hit.h"
//#include "air_hockey_baseline_agent/planner/stable_dynamics_motion.h"
#include "air_hockey_baseline_agent/planner/combinatorial_hit.h"
#include "air_hockey_baseline_agent/planner/cubic_linear_motion.h"

#include "air_hockey_baseline_agent/transformations.h"
#include "air_hockey_baseline_agent/null_space_optimizer.h"
#include "air_hockey_baseline_agent/observer.h"

namespace air_hockey_baseline_agent {

struct TrajectoryGenerator {
	TrajectoryGenerator(std::string ns, EnvironmentParams data,
			Observer *observer, double rate);
	~TrajectoryGenerator();

	CombinatorialHit *combinatorialHit;
	CubicLinearMotion *cubicLinearMotion;
	iiwas_kinematics::Kinematics *kinematics;
	Transformations *transformations;
	NullSpaceOptimizer *optimizer;
};

}

#endif /* AIRHOCKEY_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_AIR_HOCKEY_BASELINE_AGENT_TRAJECTORY_GENERATOR_H_ */
