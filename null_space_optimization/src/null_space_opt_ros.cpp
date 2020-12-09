//
// Created by puze on 07.12.20.
//

#include "null_space_opt_ros.h"

using namespace null_space_optimization;

NullSpaceOptimizerROS::NullSpaceOptimizerROS(Kinematics &kinematics) : optimizer_(kinematics), nh_("/"){

    cmdPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 1);

    weights_ << 40., 40., 20., 40., 10., 10., 10.;
    string ns = ros::this_node::getNamespace();
    string ns_prefix;
    if (ns == "/front_iiwa") {
        ns_prefix = 'F';
    } else if (ns == "/back_iiwa") {
        ns_prefix = 'B';
    } else {
        ROS_ERROR_STREAM("Run the node under the namespace: front_iiwa / back_iiwa");
        return;
    }

    jointTrajectoryPoint_.positions.resize(NUM_OF_JOINTS);
    jointTrajectoryPoint_.velocities.resize(NUM_OF_JOINTS);

    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_1");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_2");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_3");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_4");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_5");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_6");
    jointTrajectoryCmd_.joint_names.push_back(ns_prefix + "_joint_7");

}

void NullSpaceOptimizerROS::update() {

}

void NullSpaceOptimizerROS::cartesianCmdCallback(const CartersianTrajectory::ConstPtr &msg) {
    ROS_INFO_STREAM("Enter Cartesian Cmd Callback");
    timeStep_ = msg->time_step_size;
    xDes_.x() = msg->position.x;
    xDes_.y() = msg->position.y;
    xDes_.z() = msg->position.z;

    dxDes_.x() = msg->velocity.x;
    dxDes_.y() = msg->velocity.y;
    dxDes_.z() = msg->velocity.z;
}

void NullSpaceOptimizerROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    ROS_INFO_STREAM("Enter Joint State Callback");
    if (msg->position.size() == 7) {
        qCur_[0] = msg->position[0];
        qCur_[1] = msg->position[1];
        qCur_[2] = msg->position[2];
        qCur_[3] = msg->position[3];
        qCur_[4] = msg->position[4];
        qCur_[5] = msg->position[5];
        qCur_[6] = msg->position[6];
    }
}

void NullSpaceOptimizerROS::cmdCallback(const CartersianTrajectory::ConstPtr &msgCartTraj,
                                        const sensor_msgs::JointState::ConstPtr &msgJointState) {
    ROS_INFO_STREAM("Enter Synchronizer");
    cartesianCmdCallback(msgCartTraj);
    jointStateCallback(msgJointState);
    if (optimizer_.SolveQP(xDes_, dxDes_, qCur_, weights_, timeStep_, qNext_, dqNext_)) {
        generateTrajectoryCommand();
        cmdPub_.publish(jointTrajectoryCmd_);
    }
}

void NullSpaceOptimizerROS::generateTrajectoryCommand() {
    for (int i = 0; i < NUM_OF_JOINTS; ++i) {
        jointTrajectoryPoint_.positions[i] = qNext_[i];
        jointTrajectoryPoint_.velocities[i] = dqNext_[i];
    }
    jointTrajectoryPoint_.time_from_start = ros::Duration(timeStep_);

    jointTrajectoryCmd_.header.stamp = ros::Time::now();
    jointTrajectoryCmd_.points.clear();
    jointTrajectoryCmd_.points.push_back(jointTrajectoryPoint_);
}
