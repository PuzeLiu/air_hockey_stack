//
// Created by puze on 06.12.20.
//

#include <chrono>
#include <iostream>

#include "optimizer.h"
#include "null_space_opt_ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>

using namespace std;
using namespace null_space_optimization;
using namespace iiwas_kinematics;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "null_space_optimizer");
    ros::NodeHandle nh("/");
    ros::Rate rate(100);
    ros::Publisher jointTrajectoryPub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 1);
    ros::Subscriber jointStateSub;

    bool closeLoop = false;

    // Define Kinematics
    Vector3d tcp_ee(0., 0., 0.5);
    Quaterniond tcpQuat(1., 0., 0., 0.);
    Kinematics kinematics(tcp_ee, tcpQuat);
    // Define Bezier Planner
    Vector2d boundLower, boundUpper;
    double height = 0.3;
    boundLower << 0.6, -0.5;
    boundUpper << 2.2, 0.5;
    BezierCurve2D bezierPlanner(boundLower, boundUpper, height);
    // Define ROS interface
    NullSpaceOptimizerROS optimizerRos(kinematics, bezierPlanner, closeLoop);

    Kinematics::JointArrayType qDes, dqDes;
    trajectory_msgs::JointTrajectory initMsg;
    initMsg = optimizerRos.jointTrajCmdMsg;
    trajectory_msgs::JointTrajectoryPoint msgPoint;

    // Go to initial position
    Vector3d xStart(0.45, 0.0, height);
    Vector3d gc(1., -1., 1);
    Quaterniond quatStart(0., 0., -1., 0.);
    quatStart.normalize();
    double psi = -60/180.*M_PI;
    if (kinematics.inverseKinematics(xStart, quatStart, gc, psi, qDes)){
        cout << qDes << endl;
    } else{
        return -1;
    }
    dqDes << 0., 0., 0., 0., 0., 0., 0.;
    double time_from_start = 3.0;
    msgPoint = optimizerRos.generatePoint(qDes, dqDes, time_from_start);
    initMsg.points.push_back(msgPoint);
    initMsg.header.stamp = ros::Time::now();
    jointTrajectoryPub.publish(initMsg);
    ros::Duration(time_from_start + 1.0).sleep();

    if(closeLoop){
        jointStateSub = nh.subscribe("joint_state", 1, &NullSpaceOptimizerROS::jointStateCallback, &optimizerRos);
    } else{
        optimizerRos.setQInit(qDes);
    }

    // Hitting Movement
    Vector2d xDes, dxDes;
    xDes << 0.8, 0.2;
    dxDes << 1.5, 0.0;
    auto start = chrono::high_resolution_clock::now();
    if(optimizerRos.startBeizerHit(xDes, dxDes, rate.expectedCycleTime().toSec())){
        jointTrajectoryPub.publish(optimizerRos.jointTrajCmdMsg);
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Trajectory Optimization Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 1.e6 << "ms\n";
    ROS_INFO_STREAM("Hit Time: "<< optimizerRos.bezier.getHitTime());
    ROS_INFO_STREAM("Stop Time: "<< optimizerRos.bezier.getStopTime());
    ROS_INFO_STREAM("Total Points: "<< optimizerRos.jointTrajCmdMsg.points.size());
    ros::Duration(optimizerRos.bezier.getStopTime()).sleep();

    jointTrajectoryPub.publish(initMsg);
    ros::Duration(time_from_start).sleep();


//    ros::Publisher jointPointPub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("points", 1);
//    start = chrono::high_resolution_clock::now();
//    for (int i = 0; i < optimizerRos.jointTrajCmdMsg.points.size(); ++i) {
//        jointPointPub.publish(optimizerRos.jointTrajCmdMsg.points[i]);
//        ROS_INFO_STREAM("Publish");
//        rate.sleep();
//    }
//    finish = chrono::high_resolution_clock::now();
//    cout << "Trajectory Publishing Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 1.e6 << "ms\n";

    return 0;
}