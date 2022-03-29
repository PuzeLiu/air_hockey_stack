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

#include "air_hockey_referee/gazebo_referee.h"

using namespace ros;

namespace air_hockey_baseline_agent{

GazeboReferee::GazeboReferee(NodeHandle nh) : Referee(nh) {
    clientResetGazeboPuck = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

GazeboReferee::~GazeboReferee() {

}

bool GazeboReferee::resetPuck(std::string* msg, bool onHome) {
	gazebo_msgs::SetModelState::Request puckStateReq;
	gazebo_msgs::SetModelState::Response puckStateRes;
	puckStateReq.model_state.model_name = "puck";
	puckStateReq.model_state.reference_frame = "air_hockey_table::Table";
	puckStateReq.model_state.pose.position.z = 0.01;
	Eigen::Vector2d vec;
	vec.setRandom();

	puckStateReq.model_state.pose.position.y = vec.y() * (tableWidth / 2 - puckRadius - 0.1);

	if (onHome){
		puckStateReq.model_state.pose.position.x = -1 * (vec.x() * 0.3 + (tableLength / 2 - 0.4));
	} else {
		puckStateReq.model_state.pose.position.x = vec.x() * 0.3 + (tableLength / 2 - 0.4);
	}

	clientResetGazeboPuck.call(puckStateReq, puckStateRes);

	if(msg != nullptr && !puckStateRes.success) {
		*msg = puckStateRes.status_message;
	}

	return puckStateRes.success;
}

void GazeboReferee::update() {
    Referee::update();
    checkMallet();
    if (gameStatusMsg.status == GameStatus::PAUSE){
        ros::Duration(5.0).sleep();
        if (isPuckOnTable()) {
            gameStatusMsg.status = GameStatus::START;
            statusPub.publish(gameStatusMsg);
            ROS_INFO_STREAM("Game Restarted");
        }
    }
}

    void GazeboReferee::checkMallet() {
        if (gameStatusMsg.status == GameStatus::START) {
            tfMalletF = tfBuffer.lookupTransform("Table", "F_striker_mallet_tip", ros::Time(0), ros::Duration(1.0));
            tf2::fromMsg(tfMalletF.transform.rotation, quatTmp);
            rotMatTmp.setRotation(quatTmp);

            if (abs(tfMalletF.transform.translation.x) > (tableLength / 2 - malletRadius) ||
                abs(tfMalletF.transform.translation.y) > (tableWidth / 2 - malletRadius)) {
                ROS_WARN_STREAM("[Referee]: Detect exceptional position on F_striker_mallet_tip, Game Stop");
                gameStatusMsg.status = GameStatus::STOP;
                gameStatusMsg.score_away = 0;
                gameStatusMsg.score_home = 0;
                statusPub.publish(gameStatusMsg);
            }

            /*tfMalletB = tfBuffer.lookupTransform("Table", "B_striker_mallet_tip", ros::Time(0), ros::Duration(1.0));
            tf2::fromMsg(tfMalletB.transform.rotation, quatTmp);
            rotMatTmp.setRotation(quatTmp);

            if (abs(tfMalletB.transform.translation.x) > (tableLength / 2 - malletRadius) ||
                abs(tfMalletB.transform.translation.y) > (tableWidth / 2 - malletRadius)) {
                ROS_WARN_STREAM("[Referee]: Detect exceptional position on B_striker_mallet_tip, Game Stop");
                gameStatusMsg.status = GameStatus::STOP;
                gameStatusMsg.score_away = 0;
                gameStatusMsg.score_home = 0;
                statusPub.publish(gameStatusMsg);
            }*/
        }
    }

}