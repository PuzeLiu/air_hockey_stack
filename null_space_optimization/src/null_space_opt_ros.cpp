//
// Created by puze on 07.12.20.
//

#include "null_space_opt_ros.h"

using namespace null_space_optimization;

NullSpaceOptimizerROS::NullSpaceOptimizerROS(Kinematics &kinematics,
                                             BezierCurve2D &bezier,
                                             bool closeLoop) : kinematics_(kinematics),
                                                               optimizer_(kinematics),
                                                               bezier(bezier){

    weights_ << 40., 40., 20., 40., 10., 10., 10.;
    closeLoop_ = closeLoop;
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

    jointTrajCmdMsg.joint_names.push_back(ns_prefix + "_joint_1");
    jointTrajCmdMsg.joint_names.push_back(ns_prefix + "_joint_2");
    jointTrajCmdMsg.joint_names.push_back(ns_prefix + "_joint_3");
    jointTrajCmdMsg.joint_names.push_back(ns_prefix + "_joint_4");
    jointTrajCmdMsg.joint_names.push_back(ns_prefix + "_joint_5");
    jointTrajCmdMsg.joint_names.push_back(ns_prefix + "_joint_6");
    jointTrajCmdMsg.joint_names.push_back(ns_prefix + "_joint_7");

}

bool NullSpaceOptimizerROS::startBeizerHit(const Vector2d &xHit, Vector2d vHit, double stepSize) {
    Vector3d xCur;
    kinematics_.ForwardKinematics(qCur_, xCur);
    Vector2d xStart = xCur.block<2, 1>(0, 0);
    bezier.fit(xStart, xHit, vHit);
    vector<null_space_optimization::CartersianTrajectory> traj = bezier.getTrajectory(stepSize);
    vector<null_space_optimization::CartersianTrajectory>::iterator iter;
    reset();
    for (iter = traj.begin(); iter != traj.end(); iter++) {
        xDes_.x() = iter->position.x;
        xDes_.y() = iter->position.y;
        xDes_.z() = iter->position.z;
        dxDes_.x() = iter->velocity.x;
        dxDes_.y() = iter->velocity.y;
        dxDes_.z() = iter->velocity.z;
        if (optimizer_.solveQP(xDes_, dxDes_, qCur_, weights_, stepSize, qNext_, dqNext_)) {
            if (!closeLoop_) {
                qCur_ = qNext_;
            }
            time_from_start += stepSize;
            generatePoint(qNext_, dqNext_, time_from_start);
            jointTrajCmdMsg.points.push_back(jointTrajectoryPoint_);
        }
        else{
            return false;
        }
    }
    return true;
}


void NullSpaceOptimizerROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
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


trajectory_msgs::JointTrajectoryPoint
NullSpaceOptimizerROS::generatePoint(Kinematics::JointArrayType qDes, Kinematics::JointArrayType dqDes,
                                     double time_from_start) {
    for (int i = 0; i < NUM_OF_JOINTS; ++i) {
        jointTrajectoryPoint_.positions[i] = qDes[i];
        jointTrajectoryPoint_.velocities[i] = dqDes[i];
    }
    jointTrajectoryPoint_.time_from_start = ros::Duration(time_from_start);
    return jointTrajectoryPoint_;
}

bool NullSpaceOptimizerROS::setQInit(Kinematics::JointArrayType qInit) {
    if (!closeLoop_) {
        qCur_ = qInit;
        return true;
    } else {
        ROS_ERROR_STREAM("Failed to set Initial joint position in close loop optimization");
        return false;
    }
}

void NullSpaceOptimizerROS::reset() {
    jointTrajCmdMsg.points.clear();
    time_from_start = 0.0;
}