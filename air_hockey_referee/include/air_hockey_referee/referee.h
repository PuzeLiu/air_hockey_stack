/*
 * MIT License
 * Copyright (c) 2020 Davide Tateo, Puze Liu
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

#ifndef REFEREE_H
#define REFEREE_H

#include <random>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include "air_hockey_referee/GameStatus.h"
#include "air_hockey_referee/StartGame.h"
#include "air_hockey_referee/StopGame.h"
#include "air_hockey_referee/PauseGame.h"
#include "air_hockey_referee/ResetRobot.h"

namespace air_hockey_baseline_agent{
    enum GameStatus {
        STOP = 0,
        PAUSE = 1,
		START = 2,
    };
class Referee {
public:
    Referee(ros::NodeHandle nh);
    virtual ~Referee();

    void update();

protected:
    virtual bool resetPuck(std::string* msg = nullptr, bool onHome=true) = 0;
    bool isPuckOnTable();

private:
    bool serviceStartCallback(air_hockey_referee::StartGame::Request &req, air_hockey_referee::StartGame::Response &res);
    bool servicePauseCallback(air_hockey_referee::PauseGame::Request &req, air_hockey_referee::PauseGame::Response &res);
    bool serviceStopCallback(air_hockey_referee::StopGame::Request &req, air_hockey_referee::StopGame::Response &res);
    bool serviceResetCallback(air_hockey_referee::ResetRobot::Request &req, air_hockey_referee::ResetRobot::Response &res);

protected:
    double tableLength, tableWidth, goalWidth, puckRadius, malletRadius, pauseRegionLength;

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher statusPub;
    ros::ServiceServer serviceStart, serviceStop, servicePause, serviceReset;

    geometry_msgs::TransformStamped tfPuck, tfPuckPrev;
    air_hockey_referee::GameStatus gameStatusMsg;

	std::mt19937 gen;
	std::uniform_int_distribution<int> dist;
};
}

#endif //REFEREE_H
