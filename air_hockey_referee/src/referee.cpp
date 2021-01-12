#include "air_hockey_referee/referee.h"

using namespace AirHockey;

Referee::Referee(ros::NodeHandle nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {
    statusPub_ = nh_.advertise<air_hockey_referee::GameStatus>("game_status", 1);
    serviceStart_ = nh_.advertiseService("start_game", &Referee::serviceStartCallback, this);
    servicePause_ = nh_.advertiseService("pause_game", &Referee::servicePauseCallback, this);
    servicePause_ = nh_.advertiseService("stop_game", &Referee::serviceStopCallback, this);
    serviceReset_ = nh_.advertiseService("reset_robot", &Referee::serviceResetCallback, this);
    clientResetGazeboPuck_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    try {
        tfTable_ = tfBuffer_.lookupTransform("world", "Table", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Could not transform from world to Table: " << exception.what());
        ros::shutdown();
    }

    tableLength_ = 1.96;
    tableWidth_ = 1.04;
    goalWidth_ = 0.25;

    puckRadius_ = 0.03165;

    gameStatusMsg_.status = GameStatus::STOP;
    gameStatusMsg_.score_home = 0;
    gameStatusMsg_.score_away = 0;

    stampPrev_ = ros::Time::now();
}

Referee::~Referee() {

}

void Referee::update() {
    ros::spinOnce();
    try {
        tfPuck_ = tfBuffer_.lookupTransform("Table", "Puck", ros::Time(0), ros::Duration(1.0));
        if (ros::Time::now() - tfPuck_.header.stamp > ros::Duration(0.2)) {
            gameStatusMsg_.status = GameStatus::PAUSE;
            statusPub_.publish(gameStatusMsg_);
        } else if (tfPuck_.header.stamp - stampPrev_ > ros::Duration(0.0)) {
            stampPrev_ = tfPuck_.header.stamp;
            if (gameStatusMsg_.status == GameStatus::START) {
//                ROS_INFO_STREAM("Check Goal");
                if (tfPuck_.transform.translation.x < -(tableLength_ / 2 - puckRadius_ + 1e-2) &&
                    abs(tfPuck_.transform.translation.y) < goalWidth_ / 2) {
                    ROS_INFO_STREAM("Back Goal");
                    gameStatusMsg_.status = GameStatus::PAUSE;
                    gameStatusMsg_.score_away += 1;
                    statusPub_.publish(gameStatusMsg_);
                    ROS_INFO_STREAM(gameStatusMsg_);
                } else if (tfPuck_.transform.translation.x > (tableLength_ / 2 - puckRadius_ + 1e-2) &&
                           abs(tfPuck_.transform.translation.y) < goalWidth_ / 2) {
                    ROS_INFO_STREAM("Front Goal");
                    gameStatusMsg_.status = GameStatus::PAUSE;
                    gameStatusMsg_.score_home += 1;
                    statusPub_.publish(gameStatusMsg_);
                    ROS_INFO_STREAM(gameStatusMsg_);
                }
            }
        }
    } catch (tf2::TransformException &ex) {
        ROS_INFO_STREAM(ex.what());
    }
}

bool Referee::serviceStartCallback(air_hockey_referee::StartGame::Request &req,
                                   air_hockey_referee::StartGame::Response &res) {
    if (gameStatusMsg_.status == GameStatus::START){
        res.success = false;
        res.msg = "StartGame service fail, already started";
        return true;
    } else {
        gameStatusMsg_.status = GameStatus::START;
        statusPub_.publish(gameStatusMsg_);
        res.success = true;
        res.msg = "Game started";
        return true;
    }
}

bool Referee::servicePauseCallback(air_hockey_referee::PauseGame::Request &req,
                                   air_hockey_referee::PauseGame::Response &res) {
    if (gameStatusMsg_.status != GameStatus::START){
        res.success = false;
        res.msg = "PauseGame service fail, already paused";
        return true;
    } else {
        gameStatusMsg_.status = GameStatus::PAUSE;
        statusPub_.publish(gameStatusMsg_);
        res.success = true;
        res.msg = "Game paused";
        return true;
    }
}

bool Referee::serviceStopCallback(air_hockey_referee::StopGame::Request &req,
                                  air_hockey_referee::StopGame::Response &res) {
    if (gameStatusMsg_.status == GameStatus::STOP){
        res.success = false;
        res.msg = "StopGame service fail, already paused";
        return true;
    } else {
        gameStatusMsg_.status = GameStatus::STOP;
        statusPub_.publish(gameStatusMsg_);
        res.success = true;
        res.msg = "Game stopped";
        return true;
    }
}


bool Referee::serviceResetCallback(air_hockey_referee::ResetGazeboPuck::Request &req,
                                   air_hockey_referee::ResetGazeboPuck::Response &res) {
    gazebo_msgs::SetModelState::Request puckStateReq;
    gazebo_msgs::SetModelState::Response puckStateRes;
    puckStateReq.model_state.model_name = "puck";
    puckStateReq.model_state.reference_frame = "air_hockey_table::t_base";
    puckStateReq.model_state.pose.position.z = 0.0565;
    Eigen::Vector2d vec;
    vec.setRandom();
    puckStateReq.model_state.pose.position.x = vec.x() * (tableLength_ - 2 * puckRadius_) / 2;
    puckStateReq.model_state.pose.position.y = vec.y() * (tableWidth_ - 2 * puckRadius_) / 2;

    clientResetGazeboPuck_.call(puckStateReq, puckStateRes);
    res.success = puckStateRes.success;
    res.msg = puckStateRes.status_message;
    return true;
}


