#include "agent.h"

using namespace tactical_agent;

Agent::Agent(ros::NodeHandle nh, double rate) : nh_(nh), rate_(rate), observer_(nh, ros::Rate(rate)){
    universalJointHeight_ = 0.165;

    kinematics_ = new iiwas_kinematics::Kinematics(Vector3d(0.0, 0.0, 0.515),
                                                   Quaterniond(1.0, 0.0, 0.0, 0.0));
    bezierHit_ = new BezierHit(Vector2d(0.58, -0.45),
                               Vector2d(2.43, 0.45),
                               rate_,
                               universalJointHeight_);
    optimizer_ = new NullSpaceOptimizer(kinematics_, &observer_, false);

    jointTrajectoryPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 1);

    xHome_ << 0.6, 0.0;
    xGoal_ << 2.48, 0.0;

    puckRadius_ = 0.03165;
    malletRadius_ = 0.048;

    cartTrajectory_.joint_names.push_back("x");
    cartTrajectory_.joint_names.push_back("y");
    cartTrajectory_.joint_names.push_back("z");

    qHome_ << -1.08895, -1.17309, 1.61066, -1.92931, 0.0767975, 0.902649, 0.820094;

    std::string ns_prefix;
    if (nh.getNamespace() == "/iiwa_front") {
        ns_prefix = 'F';
    } else if (nh.getNamespace() == "/iiwa_back") {
        ns_prefix = 'B';
    } else {
        ROS_ERROR_STREAM("Run the node under the namespace: iiwa_front / iiwa_back");
    }
    jointTrajectory_.joint_names.push_back(ns_prefix + "_joint_1");
    jointTrajectory_.joint_names.push_back(ns_prefix + "_joint_2");
    jointTrajectory_.joint_names.push_back(ns_prefix + "_joint_3");
    jointTrajectory_.joint_names.push_back(ns_prefix + "_joint_4");
    jointTrajectory_.joint_names.push_back(ns_prefix + "_joint_5");
    jointTrajectory_.joint_names.push_back(ns_prefix + "_joint_6");
    jointTrajectory_.joint_names.push_back(ns_prefix + "_joint_7");

    jointViaPoint_.positions.resize(iiwas_kinematics::NUM_OF_JOINTS);
    jointViaPoint_.velocities.resize(iiwas_kinematics::NUM_OF_JOINTS);

}

Agent::~Agent() {
    delete kinematics_;
    delete bezierHit_;
}

void Agent::gotoHome() {
    jointTrajectory_.points.clear();
    for (int i = 0; i < 7; ++i) {
        jointViaPoint_.positions[i] = qHome_[i];
        jointViaPoint_.velocities[i] = 0.;
    }
    jointViaPoint_.time_from_start = ros::Duration(3.0);
    jointTrajectory_.header.stamp = ros::Time::now();
    jointTrajectory_.points.push_back(jointViaPoint_);
    jointTrajectoryPub_.publish(jointTrajectory_);
    ros::Duration(3.0 + 1.0).sleep();
}

void Agent::update() {
    observationState_ = observer_.getObservation();
    updateTactic();
    if(generateTrajectory()) {
        gotoHome();
    }
}

void Agent::updateTactic() {
    tacticState_ = Tactics::SMASH;
}

bool Agent::generateTrajectory() {
    if (tacticState_ == Tactics::SMASH) {
        cartTrajectory_.points.clear();

        iiwas_kinematics::Kinematics::JointArrayType qCur;
        for (size_t row = 0; row < observationState_.jointState.position.size() - 2; row++) {
            qCur[row] = observationState_.jointState.position[row];
        }
        Vector3d xCur;
        kinematics_->forwardKinematics(qCur, xCur);
        Vector2d xCur2d = xCur.block<2, 1>(0, 0);
        Vector2d puckCur2d(observationState_.puckPos.transform.translation.x,
                           observationState_.puckPos.transform.translation.y);
        double vHitMag=1.5;

        Vector2d vHit = (xGoal_ - puckCur2d).normalized();
        Vector2d xHit = puckCur2d - vHit * (puckRadius_ + malletRadius_);

        vHit *= vHitMag;

        for (size_t i=0; i<10; ++i){
            ROS_INFO_STREAM(xCur2d);
            ROS_INFO_STREAM(xHit);
            ROS_INFO_STREAM(vHit);
            if(!bezierHit_->plan(xCur2d, xHit, vHit, cartTrajectory_)){
                return false;
            }
            jointTrajectory_.points.clear();
            if(optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)){
                jointTrajectoryPub_.publish(jointTrajectory_);
                ros::Duration(jointTrajectory_.points.back().time_from_start.toSec() + 1).sleep();
                return true;
            }
            vHit *= .8;
        }
        return false;
    }
    return false;
}
