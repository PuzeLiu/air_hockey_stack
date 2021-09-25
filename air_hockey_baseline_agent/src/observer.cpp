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

#include "air_hockey_baseline_agent/observer.h"

using namespace air_hockey_baseline_agent;

Observer::Observer(ros::NodeHandle& nh, std::string controllerName, double defendLine, int n_joints) :  puckTracker(nh, defendLine){
    jointSub = nh.subscribe(controllerName + "/state", 1, &Observer::jointStateCallback, this);
    refereeSub = nh.subscribe("/air_hockey_referee/game_status", 1, &Observer::refereeStatusCallback, this);
    maxPredictionTime = puckTracker.getMaxPredictionTime() - 1e-6;
    statusChanged = true;
	observation.jointPosition.resize(n_joints);
	observation.jointVelocity.resize(n_joints);
	observation.jointDesiredPosition.resize(n_joints);
	observation.jointDesiredVelocity.resize(n_joints);
}

void Observer::start(){
    puckTracker.start();
}

Observer::~Observer() {
}

void Observer::jointStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {
    for (int i = 0; i < msg->joint_names.size(); ++i) {
        observation.jointPosition[i] = msg->actual.positions[i];
        observation.jointVelocity[i] = msg->actual.velocities[i];
        observation.jointDesiredPosition[i] = msg->desired.positions[i];
        observation.jointDesiredVelocity[i] = msg->desired.velocities[i];
    }
}

const ObservationState& Observer::getObservation() {
    observation.puckPredictedState = puckTracker.getPredictedState();
    observation.puckEstimatedState = puckTracker.getEstimatedState();
    ros::spinOnce();
    observation.stamp = ros::Time::now();
    return observation;
}

void Observer::refereeStatusCallback(const air_hockey_referee::GameStatus::ConstPtr &msg) {
	auto newStatus = *msg;
	statusChanged = statusChanged || (newStatus.status != observation.gameStatus.status);
    observation.gameStatus = newStatus;
}
