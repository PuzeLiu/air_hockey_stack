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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>
#include <thread>

#include "air_hockey_puck_tracker/PuckTracker.hpp"


using namespace air_hockey_baseline_agent;
using namespace std;

void tfdata(ros::Rate rate) {
    rosbag::Bag bag;
    bag.open("/home/airhockey/catkin_ws/src/air_hockey_stack/air_hockey_puck_tracker/data/2021-06-11-12-02-05.bag");
    static tf2_ros::TransformBroadcaster br;
    //
    for(rosbag::MessageInstance const m: rosbag::View(bag)){
        tf2_msgs::TFMessageConstPtr i = m.instantiate<tf2_msgs::TFMessage>();
        if (i != nullptr) {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.transform = i->transforms.data()->transform;
            transformStamped.child_frame_id = i->transforms.data()->child_frame_id;
            transformStamped.header.frame_id = i->transforms.data()->header.frame_id;
            transformStamped.header.stamp = ros::Time::now();
            br.sendTransform(transformStamped);
        }
        rate.sleep();
    }
    bag.close();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "puck_tracker");
    ros::NodeHandle nh("/");
    ros::Rate rate(120);

    PuckTracker puckTracker(nh, 0.0);
    PuckPredictedState state_predict;
    PuckState state_estimate;

    puckTracker.start();
    while (ros::ok()){
        state_predict = puckTracker.getPredictedState(true, false);
		state_estimate = puckTracker.getEstimatedState(false);
        rate.sleep();
    }

    nh.shutdown();
    return 0;
}

