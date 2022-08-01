#include "air_hockey_referee/referee.h"

using namespace std;
using namespace air_hockey_msgs;
using namespace air_hockey_baseline_agent;

Referee::Referee(ros::NodeHandle nh) : nh(nh), tfBuffer(), tfListener(tfBuffer), dist(0, 1){
    statusPub = nh.advertise<air_hockey_msgs::GameStatus>("game_status", 1);
    serviceStart = nh.advertiseService("start_game", &Referee::serviceStartCallback, this);
    servicePause = nh.advertiseService("pause_game", &Referee::servicePauseCallback, this);
    serviceStop = nh.advertiseService("stop_game", &Referee::serviceStopCallback, this);
    serviceReset = nh.advertiseService("reset_robot", &Referee::serviceResetCallback, this);

    try {
        tfBuffer.lookupTransform("world", "Table", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Could not transform from world to Table: " << exception.what());
        ros::shutdown();
    }

    nh.param("/air_hockey/table_length", tableLength, 1.956);
    nh.param("/air_hockey/table_width", tableWidth, 1.042);
    nh.param("/air_hockey/goal_width", goalWidth, 0.25);
    nh.param("/air_hockey/puck_radius", puckRadius, 0.03165);
    nh.param("/air_hockey/mallet_radius", malletRadius, 0.04815);
    nh.param("/air_hockey/pause_region_length", pauseRegionLength, 0.2);

    gameStatusMsg.status = GameStatus::STOP;
    gameStatusMsg.score_home = 0;
    gameStatusMsg.score_away = 0;

}

Referee::~Referee() {

}

void Referee::update() {
    ros::spinOnce();
    try {
        tfPuck = tfBuffer.lookupTransform("Table", "Puck", ros::Time(0));
        auto dt = (tfPuck.header.stamp - tfPuckPrev.header.stamp).toSec();
        if (ros::Time::now() - tfPuck.header.stamp > ros::Duration(2.0)) {
            ROS_INFO_STREAM("Puck was not detected for 2 seconds");
            //gameStatusMsg.status = GameStatus::STOP;
            //statusPub.publish(gameStatusMsg);
        } else if (dt > 0.0) {
            if (gameStatusMsg.status == GameStatus::START) {
                Eigen::Vector3d v;
                v.x() =  (tfPuck.transform.translation.x - tfPuckPrev.transform.translation.x) / dt;
                v.y() =  (tfPuck.transform.translation.y - tfPuckPrev.transform.translation.y) / dt;
                v.z() =  (tfPuck.transform.translation.z - tfPuckPrev.transform.translation.z) / dt;

                if (abs(tfPuck.transform.translation.z) > 0.05){
                    ROS_INFO_STREAM("Detect Puck is not on the Table, Game Paused");
                    gameStatusMsg.status = GameStatus::PAUSE;
                    statusPub.publish(gameStatusMsg);
                    std::string msg;
                    resetPuck(&msg, true);
                } else if (v.norm() < 0.01 && abs(tfPuck.transform.translation.x) < pauseRegionLength / 2){
                    ROS_INFO_STREAM("Detect Puck is not reachable for both Robot, Game Paused");
                    gameStatusMsg.status = GameStatus::PAUSE;
                    statusPub.publish(gameStatusMsg);
	                auto onHome = dist(gen);
	                std::string msg;
	                resetPuck(&msg, onHome);
                } else {
                    if (tfPuck.transform.translation.x < -(tableLength / 2 + 1e-2) &&
                        abs(tfPuck.transform.translation.y) < goalWidth / 2) {
                        ROS_INFO_STREAM("Back Goal");
                        gameStatusMsg.status = GameStatus::PAUSE;
                        gameStatusMsg.score_away += 1;
                        statusPub.publish(gameStatusMsg);
                        ROS_INFO_STREAM(gameStatusMsg);
                        std::string msg;
                        resetPuck(&msg, false);
                    } else if (tfPuck.transform.translation.x > (tableLength / 2 + 1e-2) &&
                               abs(tfPuck.transform.translation.y) < goalWidth / 2) {
                        ROS_INFO_STREAM("Front Goal");
                        gameStatusMsg.status = GameStatus::PAUSE;
                        gameStatusMsg.score_home += 1;
                        statusPub.publish(gameStatusMsg);
                        ROS_INFO_STREAM(gameStatusMsg);
	                    std::string msg;
                        resetPuck(&msg, true);
                    }
                }
            }
            tfPuckPrev = tfPuck;
        }
    } catch (tf2::TransformException &ex) {
    }
}

bool Referee::serviceStartCallback(StartGame::Request &req,
                                   StartGame::Response &res) {
    if (gameStatusMsg.status == GameStatus::START){
        res.success = false;
        res.msg = "StartGame service fail, already started";
        return true;
    } else {
        if (isPuckOnTable()){
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

bool Referee::servicePauseCallback(PauseGame::Request &req,
                                   PauseGame::Response &res) {
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

bool Referee::serviceStopCallback(StopGame::Request &req,
                                  StopGame::Response &res) {
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

bool Referee::serviceResetCallback(ResetRobot::Request &req,
                                   ResetRobot::Response &res) {
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

bool Referee::isPuckOnTable() {
    tfPuck = tfBuffer.lookupTransform("Table", "Puck", ros::Time(0), ros::Duration(1.0));

    if (abs(tfPuck.transform.translation.x) < (tableLength / 2 + 1e-2) &&
        abs(tfPuck.transform.translation.y) < (tableWidth / 2 + 1e-2) &&
        abs(tfPuck.transform.translation.z) < 0.05){
        return true;
    } else{
        return false;
    }
}


