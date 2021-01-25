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

namespace AirHockey{

GazeboReferee::GazeboReferee(NodeHandle nh) : Referee(nh) {
    clientResetGazeboPuck = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

GazeboReferee::~GazeboReferee() {

}

bool GazeboReferee::resetPuck(std::string* msg)
{
	gazebo_msgs::SetModelState::Request puckStateReq;
	gazebo_msgs::SetModelState::Response puckStateRes;
	puckStateReq.model_state.model_name = "puck";
	puckStateReq.model_state.reference_frame = "air_hockey_table::Table";
	puckStateReq.model_state.pose.position.z = 0.03;
	Eigen::Vector2d vec;
	vec.setRandom();
	puckStateReq.model_state.pose.position.x = vec.x() * (tableLength - 2 * puckRadius) / 2;
	puckStateReq.model_state.pose.position.y = vec.y() * (tableWidth - 2 * puckRadius) / 2;

	clientResetGazeboPuck.call(puckStateReq, puckStateRes);

	if(msg != nullptr && !puckStateRes.success) {
		*msg = puckStateRes.status_message;
	}

	return puckStateRes.success;
}

void GazeboReferee::update() {
    Referee::update();
    if (gameStatusMsg.status == GameStatus::PAUSE){
        ros::Duration(3.0).sleep();
        resetPuck(nullptr);
        if (isPuckOnTable()) {
            gameStatusMsg.status = GameStatus::START;
            statusPub.publish(gameStatusMsg);
            ROS_INFO_STREAM("Game Restarted");
        }
    }
}

}