#include "air_hockey_referee/referee.h"

using namespace AirHockey;
using namespace std;

Referee::Referee(ros::NodeHandle nh) : nh(nh), tfBuffer(), tfListener(tfBuffer) {
    statusPub = nh.advertise<air_hockey_referee::GameStatus>("game_status", 1);
    serviceStart = nh.advertiseService("start_game", &Referee::serviceStartCallback, this);
    servicePause = nh.advertiseService("pause_game", &Referee::servicePauseCallback, this);
    serviceStop = nh.advertiseService("stop_game", &Referee::serviceStopCallback, this);
    serviceReset = nh.advertiseService("reset_robot", &Referee::serviceResetCallback, this);

    try {
        tfTable = tfBuffer.lookupTransform("world", "Table", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Could not transform from world to Table: " << exception.what());
        ros::shutdown();
    }

    nh.param("/air_hockey/table_length", tableLength, 1.956);
    nh.param("/air_hockey/table_width", tableWidth, 1.042);
    nh.param("/air_hockey/goal_width", goalWidth, 0.25);
    nh.param("/air_hockey/puck_radius", puckRadius, 0.03165);

    gameStatusMsg.status = GameStatus::STOP;
    gameStatusMsg.score_home = 0;
    gameStatusMsg.score_away = 0;

    stampPrev = ros::Time::now();
}

Referee::~Referee() {

}

void Referee::update() {
    ros::spinOnce();
    try {
        tfPuck = tfBuffer.lookupTransform("Table", "Puck", ros::Time(0), ros::Duration(1.0));
        if (ros::Time::now() - tfPuck.header.stamp > ros::Duration(0.2)) {
            gameStatusMsg.status = GameStatus::PAUSE;
            statusPub.publish(gameStatusMsg);
        } else if (tfPuck.header.stamp - stampPrev > ros::Duration(0.0)) {
            stampPrev = tfPuck.header.stamp;
            if (gameStatusMsg.status == GameStatus::START) {
//                ROS_INFO_STREAM("Check Goal");
                if (tfPuck.transform.translation.x < -(tableLength / 2 + 1e-2) &&
                    abs(tfPuck.transform.translation.y) < goalWidth / 2) {
                    ROS_INFO_STREAM("Back Goal");
                    gameStatusMsg.status = GameStatus::PAUSE;
                    gameStatusMsg.score_away += 1;
                    statusPub.publish(gameStatusMsg);
                    ROS_INFO_STREAM(gameStatusMsg);
                    string msg;
                    resetPuck();
                } else if (tfPuck.transform.translation.x > (tableLength / 2 + 1e-2) &&
                           abs(tfPuck.transform.translation.y) < goalWidth / 2) {
                    ROS_INFO_STREAM("Front Goal");
                    gameStatusMsg.status = GameStatus::PAUSE;
                    gameStatusMsg.score_home += 1;
                    statusPub.publish(gameStatusMsg);
                    ROS_INFO_STREAM(gameStatusMsg);
                    resetPuck();
                }
            }
        }
    } catch (tf2::TransformException &ex) {
        ROS_INFO_STREAM(ex.what());
    }
}

bool Referee::serviceStartCallback(air_hockey_referee::StartGame::Request &req,
                                   air_hockey_referee::StartGame::Response &res) {
    if (gameStatusMsg.status == GameStatus::START){
        res.success = false;
        res.msg = "StartGame service fail, already started";
        return true;
    } else {
        tfPuck = tfBuffer.lookupTransform("Table", "Puck", ros::Time(0), ros::Duration(1.0));
        if (abs(tfPuck.transform.translation.x) < (tableLength / 2 + 1e-2) &&
            abs(tfPuck.transform.translation.y) < (tableWidth / 2 + 1e-2) &&
            abs(tfPuck.transform.translation.z) < 3e-2){
            gameStatusMsg.status = GameStatus::START;
            statusPub.publish(gameStatusMsg);
            res.success = true;
            res.msg = "Game started";
            return true;
        } else {
            res.success = false;
            res.msg = "StartGame service fail, puck is not detected on the table";
        }
    }

    return true;
}

bool Referee::servicePauseCallback(air_hockey_referee::PauseGame::Request &req,
                                   air_hockey_referee::PauseGame::Response &res) {
    if (gameStatusMsg.status != GameStatus::START){
        res.success = false;
        res.msg = "PauseGame service fail, already paused";
        return true;
    } else {
        gameStatusMsg.status = GameStatus::PAUSE;
        statusPub.publish(gameStatusMsg);
        res.success = true;
        res.msg = "Game paused";
        return true;
    }
}

bool Referee::serviceStopCallback(air_hockey_referee::StopGame::Request &req,
                                  air_hockey_referee::StopGame::Response &res) {
    if (gameStatusMsg.status == GameStatus::STOP){
        res.success = false;
        res.msg = "StopGame service fail, already stopped";
        return true;
    } else {
        gameStatusMsg.status = GameStatus::STOP;
        gameStatusMsg.score_away = 0;
        gameStatusMsg.score_home = 0;
        statusPub.publish(gameStatusMsg);
        res.success = true;
        res.msg = "Game stopped";
        return true;
    }
}

bool Referee::serviceResetCallback(air_hockey_referee::ResetRobot::Request &req,
                                   air_hockey_referee::ResetRobot::Response &res) {
	string msg;
	bool success = resetPuck(&msg);

	res.success = success;

	if(res.success) {
		res.msg = "Robot state reset";
	} else {
		res.msg = msg;
	}
    gameStatusMsg.status = GameStatus::STOP;
    statusPub.publish(gameStatusMsg);

    return true;
}


