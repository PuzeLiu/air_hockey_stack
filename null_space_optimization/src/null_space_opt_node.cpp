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
    ros::Publisher jointTrajectoryPub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 10);
    ros::Subscriber jointStateSub;

    ros::Duration(1).sleep();

    bool closeLoop = false;

    // Define Kinematics
    Vector3d tcp_ee(0., 0., 0.515);
    Quaterniond tcpQuat(1., 0., 0., 0.);
    Kinematics kinematics(tcp_ee, tcpQuat);

    Kinematics::JointArrayType qDes, dqDes;
    Vector3d xStart;
    Quaterniond quatStart;
    Vector3d gc;
    double psi;

//    qDes << -0.0006934671992886987, -0.11952562587417677, 0.08305109801173922, -1.4600705375628737, 0.00016317728948718225, 1.298426712791534, -6.717161425687976e-05;
    qDes << -0.9633389760610079, -1.1355259826143858, 1.5364214352585959, -1.9904962267807123, 0.036825509800739106, 0.9567722200198567, 0.8145943717912628;
    kinematics.forwardKinematics(qDes, xStart, quatStart);
    kinematics.getRedundancy(qDes, gc, psi);
    double height = 0.165;
//    double height = 0.28;
//    psi = -80 / 180. * M_PI;
    ROS_INFO_STREAM("Start Position: " << xStart);
    ROS_INFO_STREAM("global configuration : " << gc);
    ROS_INFO_STREAM("Redundancy : " << psi);

    // Define Bezier Planner
    Vector2d boundLower, boundUpper;
    boundLower << 0.6, -0.5;
    boundUpper << 2.2, 0.5;
    BezierCurve2D bezierPlanner(boundLower, boundUpper, height);
    // Define ROS interface
    NullSpaceOptimizerROS optimizerRos(kinematics, bezierPlanner, closeLoop);


    trajectory_msgs::JointTrajectory initMsg;
    initMsg = optimizerRos.jointTrajCmdMsg;
    trajectory_msgs::JointTrajectoryPoint msgPoint;

    // Go to initial position
    xStart<< 0.65, 0.05, height;
    if (kinematics.inverseKinematics(xStart, quatStart, gc, psi, qDes)){
        cout << qDes << endl;
    } else{
        return -1;
    }
    dqDes << 0., 0., 0., 0., 0., 0., 0.;
    double time_from_start = 5.0;
    msgPoint = optimizerRos.generatePoint(qDes, dqDes, time_from_start);
    initMsg.points.push_back(msgPoint);
    initMsg.header.stamp = ros::Time::now();
    initMsg.header.seq = 0;
    initMsg.header.frame_id = "";
    jointTrajectoryPub.publish(initMsg);
    ROS_INFO_STREAM(initMsg);
    ros::Duration(time_from_start + 1.0).sleep();

    if(closeLoop){
        jointStateSub = nh.subscribe("joint_state", 1, &NullSpaceOptimizerROS::jointStateCallback, &optimizerRos);
    } else{
        optimizerRos.setQInit(qDes);
    }

    // Hitting Movement
    Vector2d xDes, dxDes;
    xDes << 0.8 , -0.2;
    dxDes << 1.1, 0.0;
    auto start = chrono::high_resolution_clock::now();
    if(optimizerRos.startBeizerHit(xDes, dxDes, rate.expectedCycleTime().toSec())){
        jointTrajectoryPub.publish(optimizerRos.jointTrajCmdMsg);
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Trajectory Optimization Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 1.e6 << "ms\n";
    ROS_INFO_STREAM("Hit Time: "<< optimizerRos.bezier.getHitTime());
    ROS_INFO_STREAM("Stop Time: "<< optimizerRos.bezier.getStopTime());
    ROS_INFO_STREAM("Total Points: "<< optimizerRos.jointTrajCmdMsg.points.size());
    ros::Duration(optimizerRos.bezier.getStopTime() + 1.0).sleep();

    initMsg.header.stamp = ros::Time::now();
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