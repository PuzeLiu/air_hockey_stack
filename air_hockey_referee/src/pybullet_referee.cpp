/*
 * MIT License
 * Copyright (c) 2022 Davide Tateo, Puze Liu
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

#include "../include/air_hockey_referee/pybullet_referee.h"

using namespace air_hockey_baseline_agent;

PybulletReferee::PybulletReferee(ros::NodeHandle nh) : Referee(nh)
{
	clientResetPybulletPuck = nh.serviceClient<air_hockey_pybullet::ResetPuck>("/puck/reset_puck_state");
}

PybulletReferee::~PybulletReferee()
{

}

void PybulletReferee::update()
{
	Referee::update();
	checkMallet();
	if (gameStatusMsg.status == GameStatus::PAUSE)
	{
		ros::Duration(5.0).sleep();
		if (isPuckOnTable())
		{
			gameStatusMsg.status = GameStatus::START;
			statusPub.publish(gameStatusMsg);
			ROS_INFO_STREAM("Game Restarted");
		}
	}
}

void PybulletReferee::checkMallet()
{
	if (gameStatusMsg.status == GameStatus::START)
	{
		tfMalletF = tfBuffer.lookupTransform("Table", "F_striker_mallet_tip", ros::Time(0), ros::Duration(1.0));
		tf2::fromMsg(tfMalletF.transform.rotation, quatTmp);
		rotMatTmp.setRotation(quatTmp);

		if (abs(tfMalletF.transform.translation.x) > (tableLength / 2 - malletRadius - 0.02) ||
			abs(tfMalletF.transform.translation.y) > (tableWidth / 2 - malletRadius + 0.02))
		{
			ROS_WARN_STREAM("[Referee]: Detect exceptional position on F_striker_mallet_tip, Game Stop");
			gameStatusMsg.status = GameStatus::STOP;
			gameStatusMsg.score_away = 0;
			gameStatusMsg.score_home = 0;
			statusPub.publish(gameStatusMsg);
		}

		tfMalletB = tfBuffer.lookupTransform("Table", "B_striker_mallet_tip", ros::Time(0), ros::Duration(1.0));
		tf2::fromMsg(tfMalletB.transform.rotation, quatTmp);
		rotMatTmp.setRotation(quatTmp);

		if (abs(tfMalletB.transform.translation.x) > (tableLength / 2 - malletRadius - 0.02) ||
			abs(tfMalletB.transform.translation.y) > (tableWidth / 2 - malletRadius + 0.02))
		{
			ROS_WARN_STREAM("[Referee]: Detect exceptional position on B_striker_mallet_tip, Game Stop");
			gameStatusMsg.status = GameStatus::STOP;
			gameStatusMsg.score_away = 0;
			gameStatusMsg.score_home = 0;
			statusPub.publish(gameStatusMsg);
		}
	}
}

bool PybulletReferee::resetPuck(std::string* msg, bool onHome) {
	air_hockey_pybullet::ResetPuck::Request puckStateReq;
	air_hockey_pybullet::ResetPuck::Response puckStateRes;
	puckStateReq.transform.header.frame_id = "Table";
	puckStateReq.transform.transform.translation.z = 0.01;
	Eigen::Vector2d vec;
	vec.setRandom();

	puckStateReq.transform.transform.translation.y = vec.y() * (tableWidth / 2 - puckRadius - 0.1);

	if (onHome){
		puckStateReq.transform.transform.translation.x = -1 * (vec.x() * 0.3 + (tableLength / 2 - 0.4));
	} else {
		puckStateReq.transform.transform.translation.x = vec.x() * 0.3 + (tableLength / 2 - 0.4);
	}

	clientResetPybulletPuck.call(puckStateReq, puckStateRes);

	return true;
}