//
// Created by puze on 07.12.20.
//

#include "null_space_opt_ros.h"

using namespace null_space_optimization;

NullSpaceOptimizerROS::NullSpaceOptimizerROS(Kinematics kinematics) : nh_("~"), kinematics_(kinematics),
                                                                      optimizer_(kinematics){
    cmdPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 1);
//    cartPosSub_ = nh_.subscribe("cartesian_command", 1, &NullSpaceOptimizerROS::CartesianCmdCallback, this);
    jointPosSub_ = nh_.subscribe("joint_state", 1, &NullSpaceOptimizerROS::JointPositionCallback, this);

    weights_ << 40., 40., 20., 40., 10., 10., 10.;
    string ns = nh_.getNamespace();
    ROS_INFO_STREAM(ns);
    string ns_prefix;
    if (ns == "front_iiwa"){
        ns_prefix = 'F';
    } else if (ns == "back_iiwa"){
        ns_prefix = 'B';
    }

    jointTrajectoryCmd_.joint_names.resize(iiwas_kinematics::NUM_OF_JOINTS);
    jointTrajectoryCmd_.points.resize(1);
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_1");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_2");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_3");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_4");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_5");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_6");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_7");

    jointTrajectoryPoint_.positions.resize(iiwas_kinematics::NUM_OF_JOINTS);
    jointTrajectoryPoint_.velocities.resize(iiwas_kinematics::NUM_OF_JOINTS);
    jointTrajectoryPoint_.accelerations.resize(iiwas_kinematics::NUM_OF_JOINTS);

}

void NullSpaceOptimizerROS::Start() {
    ros::spinOnce();
    optimizer_.SolveQP(xDes_, dxDes_, qCur_, weights_, timeStep_, qNext_, dqNext_);

    jointTrajectoryCmd_.header.stamp = ros::Time::now();
    jointTrajectoryCmd_.points.push_back(jointTrajectoryPoint_);
    cmdPub_.publish(jointTrajectoryCmd_);

}

//void NullSpaceOptimizerROS::CartesianCmdCallback(const null_space_optim::::ConstPtr &msg) {
//    xDes_.x() = msg->points[0].transforms[0].translation.x;
//    xDes_.y() = msg->points[0].transforms[0].translation.y;
//    xDes_.z() = msg->points[0].transforms[0].translation.z;
//
//    dxDes_.x() = msg->points[0].velocities[0].linear.x;
//    dxDes_.y() = msg->points[0].velocities[0].linear.y;
//    dxDes_.z() = msg->points[0].velocities[0].linear.z;
//}

void NullSpaceOptimizerROS::JointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    qCur_[0] = msg->position[0];
    qCur_[1] = msg->position[1];
    qCur_[2] = msg->position[2];
    qCur_[3] = msg->position[3];
    qCur_[4] = msg->position[4];
    qCur_[5] = msg->position[5];
    qCur_[6] = msg->position[6];
}

void NullSpaceOptimizerROS::GenerateTrajectoryPoint(const VectorXd solution) {

//    jointTrajectoryPoint_.positions.push_back()
}
