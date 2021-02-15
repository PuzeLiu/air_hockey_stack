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

#ifndef SRC_TACTICAL_AGENT_OBSERVATION_H
#define SRC_TACTICAL_AGENT_OBSERVATION_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <iiwas_kinematics/iiwas_kinematics.h>

#include <control_msgs/JointTrajectoryControllerState.h>

#include "air_hockey_baseline_agent/data_structures.h"

namespace air_hockey_baseline_agent {
    class Observer {
    public:
        Observer(ros::NodeHandle& nh, std::string controllerName, double defendLine);

        ~Observer();

        void start();

        const ObservationState& getObservation();

        inline bool isGameStatusChanged() {
        	bool changed = statusChanged;
        	statusChanged = false;
        	return changed;
        }

        inline const double getMaxPredictionTime() {
        	return maxPredictionTime;
        }



    private:
        void jointStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
        void refereeStatusCallback(const air_hockey_referee::GameStatus::ConstPtr &msg);

    private:
        ros::Subscriber jointSub;
        ros::Subscriber refereeSub;

        air_hockey_baseline_agent::PuckTracker  puckTracker;
        double maxPredictionTime;

        ObservationState observation;
        bool statusChanged;
    };
}

#endif //SRC_TACTICAL_AGENT_OBSERVATION_H
