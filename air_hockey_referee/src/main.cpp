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

#include <ros/ros.h>
#include "air_hockey_referee/gazebo_referee.h"
#include "air_hockey_referee/real_world_referee.h"
#include "air_hockey_referee/pybullet_referee.h"

using namespace air_hockey_baseline_agent;
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "air_hockey_referee");
    ros::NodeHandle nh("~");
    ros::Rate rate(200);
    ros::Duration(1.0).sleep();

    std::string physicsEnv;
    nh.param<std::string>("physics", physicsEnv, "");
    if (physicsEnv == "gazebo"){
        GazeboReferee referee(nh);
        while (ros::ok()){
            referee.update();
            rate.sleep();
        }
    } else if (physicsEnv == "real"){
        RealWorldReferee referee(nh);
        while (ros::ok()){
            referee.update();
            rate.sleep();
        }
    } else if (physicsEnv == "pybullet"){
		PybulletReferee referee(nh);
		while (ros::ok()){
			referee.update();
			rate.sleep();
		}
	} else {
		ROS_ERROR_STREAM("Unknown parameter in /air_hockey_referee/physics, options: [gazebo, real, pybullet]");
	}
    return 0;
}

