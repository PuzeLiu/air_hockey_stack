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
    NullSpaceOptimizerROS(Kinematics& kinematics);

    void cartesianCmdCallback(const CartersianTrajectory::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void cmdCallback(const CartersianTrajectory::ConstPtr& msgCartTraj,
                     const sensor_msgs::JointState::ConstPtr& msgJointState);

    void update();

private:
    void optimize();
    void generateTrajectoryCommand();


private:
    ros::NodeHandle nh_;
    ros::Publisher cmdPub_;

    trajectory_msgs::JointTrajectory jointTrajectoryCmd_;
    trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint_;

    NullSpaceOptimizer optimizer_;

    Vector3d xDes_, dxDes_;
    Kinematics::JointArrayType qCur_, qNext_, dqNext_;

    Kinematics::JointArrayType weights_; //Diagonal Weight Matrix for Optimization

    double timeStep_;
};
}
#endif //SRC_NULL_SPACE_OPT_ROS_H
