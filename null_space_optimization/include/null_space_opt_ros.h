//
// Created by puze on 07.12.20.
//

#ifndef SRC_NULL_SPACE_OPT_ROS_H
#define SRC_NULL_SPACE_OPT_ROS_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include "optimizer.h"
#include "planner.h"
#include "null_space_optimization/CartersianTrajectory.h"


namespace null_space_optimization{
class NullSpaceOptimizerROS{
public:
    NullSpaceOptimizerROS(Kinematics& kinematics, BezierCurve2D& bezier, bool closeLoop=false);

    /**
     * Set initial joint position for open loop optimization
     * @param q_init
     */
    bool setQInit(Kinematics::JointArrayType qInit);

    trajectory_msgs::JointTrajectoryPoint generatePoint(Kinematics::JointArrayType qDes,
                                                        Kinematics::JointArrayType dqDes,
                                                        double time_from_start);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

    bool startBeizerHit(const Vector2d& xHit, const Vector2d& vHit, double stepSize);

    void reset();

public:
    trajectory_msgs::JointTrajectory jointTrajCmdMsg;
    BezierCurve2D &bezier;

private:
    NullSpaceOptimizer optimizer_;
    Kinematics &kinematics_;

    trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint_;

    Vector3d xDes_, dxDes_, xTest_;
    Kinematics::JointArrayType qCur_, qNext_, dqNext_;

    Kinematics::JointArrayType weights_; //Diagonal Weight Matrix for Optimization
    bool closeLoop_;

    double time_from_start;
};
}
#endif //SRC_NULL_SPACE_OPT_ROS_H
