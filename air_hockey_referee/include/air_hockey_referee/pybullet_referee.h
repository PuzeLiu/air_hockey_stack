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

#ifndef PYBULLET_REFEREE_H
#define PYBULLET_REFEREE_H

#include "air_hockey_referee/referee.h"
#include "air_hockey_pybullet/ResetPuck.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace air_hockey_baseline_agent{
class PybulletReferee: public Referee {
 public:
	PybulletReferee(ros::NodeHandle nh);
	virtual ~PybulletReferee();

	void update();

 protected:
	bool resetPuck(std::string* res = nullptr, bool onHome=true);
	void checkMallet();

 private:
	ros::ServiceClient clientResetPybulletPuck;
	geometry_msgs::TransformStamped tfMalletF, tfMalletB;
	tf2::Quaternion quatTmp;
	tf2::Matrix3x3 rotMatTmp;
};

}


#endif //SRC_AIR_HOCKEY_STACK_AIR_HOCKEY_REFEREE_INCLUDE_AIR_HOCKEY_REFEREE_PYBULLET_REFEREE_H_
