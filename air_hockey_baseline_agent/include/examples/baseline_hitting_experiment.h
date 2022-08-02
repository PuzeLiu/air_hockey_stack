/*
 * MIT License
 * Copyright (c) 2020-2022 Puze Liu, Davide Tateo, Piotr Kicki
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


#ifndef SRC_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_EXAMPLES_BASELINE_HITTING_EXPERIMENT_H_
#define SRC_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_EXAMPLES_BASELINE_HITTING_EXPERIMENT_H_

#include "air_hockey_baseline_agent/agent.h"
#include "air_hockey_msgs/PlannerRequest.h"
#include "air_hockey_msgs/PlannerStatus.h"
#include "air_hockey_msgs/GetHittingState.h"
#include <ros/ros.h>

namespace baseline_hitting_experiment {
class BaselineHittingExperiment {
 public:
  BaselineHittingExperiment(ros::NodeHandle nh);

  void start();

  bool planHittingTrajectory();

  bool planGettingBackTrajectory();

  void planRequestCB(air_hockey_msgs::PlannerRequestConstPtr msg);

 private:
  ros::NodeHandle nh;
  ros::ServiceClient get_hitting_state_client;
  air_hockey_baseline_agent::Agent agent;
  ros::Subscriber plannerRequestSub;
  ros::Publisher plannerStatusPub;

  trajectory_msgs::MultiDOFJointTrajectory cartTraj;
  trajectory_msgs::JointTrajectory jointTraj;
  trajectory_msgs::JointTrajectoryPoint prePlanPoint;

  air_hockey_baseline_agent::JointArrayType qStart, dqStart, ddqStart, qEnd;
  Eigen::Vector3d hitPos, hitDir, endPoint;

  bool publishStatus;

  air_hockey_msgs::PlannerStatus plannerStatusMsg;
};
}
#endif //SRC_AIR_HOCKEY_STACK_AIR_HOCKEY_BASELINE_AGENT_INCLUDE_EXAMPLES_BASELINE_HITTING_EXPERIMENT_H_
