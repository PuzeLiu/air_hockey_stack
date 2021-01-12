//
// Created by puze on 07.01.21.
//

#ifndef SRC_REFEREE_H
#define SRC_REFEREE_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include "air_hockey_referee/GameStatus.h"
#include "air_hockey_referee/StartGame.h"
#include "air_hockey_referee/StopGame.h"
#include "air_hockey_referee/PauseGame.h"
#include "air_hockey_referee/ResetGazeboPuck.h"
#include <gazebo_msgs/SetModelState.h>

namespace AirHockey{
    enum GameStatus {
        STOP = 0,
        PAUSE = 1,
		START = 2,
    };
class Referee {
public:
    Referee(ros::NodeHandle nh);
    ~Referee();

    void update();

private:
    bool serviceStartCallback(air_hockey_referee::StartGame::Request &req, air_hockey_referee::StartGame::Response &res);
    bool servicePauseCallback(air_hockey_referee::PauseGame::Request &req, air_hockey_referee::PauseGame::Response &res);
    bool serviceStopCallback(air_hockey_referee::StopGame::Request &req, air_hockey_referee::StopGame::Response &res);
    bool serviceResetCallback(air_hockey_referee::ResetGazeboPuck::Request &req, air_hockey_referee::ResetGazeboPuck::Response &res);

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Publisher statusPub_;
    ros::ServiceServer serviceStart_, serviceStop_, servicePause_, serviceReset_;
    ros::ServiceClient clientResetGazeboPuck_;

    geometry_msgs::TransformStamped tfPuck_, tfTable_;

    double tableLength_, tableWidth_, goalWidth_, puckRadius_;
    air_hockey_referee::GameStatus gameStatusMsg_;


    ros::Time stampPrev_;
};
}

#endif //SRC_REFEREE_H
