//
// Created by puze on 07.12.20.
//

#ifndef SRC_NULL_SPACE_OPT_ROS_H
#define SRC_NULL_SPACE_OPT_ROS_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include "null_space_opt.h"
#include "null_space_optimization/CartersianTrajectory.h"


namespace null_space_optimization{
class NullSpaceOptimizerROS{
public:
    NullSpaceOptimizerROS(Kinematics kinematics);

    void Start();

private:
    void GenerateTrajectoryPoint(const VectorXd solution);
    void CartesianCmdCallback(const CartersianTrajectory::ConstPtr& msg);
    void JointPositionCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher cmdPub_;
    ros::Subscriber cartPosSub_, jointPosSub_;
    trajectory_msgs::JointTrajectory jointTrajectoryCmd_;
    trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint_;
    double timeStep_;

    Kinematics& kinematics_;
    NullSpaceOptimizer optimizer_;

    Vector3d xDes_, dxDes_;
    Kinematics::JointArrayType qCur_, qNext_, dqNext_;

    Kinematics::JointArrayType weights_; //Diagonal Weight Matrix for Optimization




};
}
#endif //SRC_NULL_SPACE_OPT_ROS_H
