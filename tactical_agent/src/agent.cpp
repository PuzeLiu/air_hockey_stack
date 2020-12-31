#include "agent.h"

using namespace tactical_agent;

Agent::Agent(ros::NodeHandle nh, double rate) : nh_(nh), rate_(rate), observer_(nh, ros::Rate(rate)) {
    universalJointHeight_ = 0.165;

    kinematics_ = new iiwas_kinematics::Kinematics(Vector3d(0.0, 0.0, 0.515),
                                                   Quaterniond(1.0, 0.0, 0.0, 0.0));
    optimizer_ = new NullSpaceOptimizer(kinematics_, &observer_, false);

    bezierHit_ = new BezierHit(Vector2d(0.58, -0.47),
                               Vector2d(2.43, 0.47),
                               rate_,
                               universalJointHeight_);
    combinatorialHit_ = new CombinatorialHit(Vector2d(0.58, -0.45),
                                             Vector2d(2.43, 0.45),
                                             rate_,
                                             universalJointHeight_);

    jointTrajectoryPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
            "joint_position_trajectory_controller/command", 1);
    cartTrajectoryPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cartesian_trajectory", 1);

    xHome_ << 0.6, 0.0;
    xGoal_ << 2.48, 0.1;

    puckRadius_ = 0.03165;
    malletRadius_ = 0.048;

    cartTrajectory_.joint_names.push_back("x");
    cartTrajectory_.joint_names.push_back("y");
    cartTrajectory_.joint_names.push_back("z");

    qHome_ << 1.4568405, 1.410468, -1.6053885, -1.0593628, 0.34509358, 1.6828097, 0.;
//    Vector3d xHome3d, gc;
//    xHome3d << 0.6, 0.0, universalJointHeight_;
//    gc << -1., -1., 1.;
//    Quaterniond quatHome;
//    quatHome = AngleAxisd(M_PI, Vector3d::UnitY()) * AngleAxisd(M_PI/4, Vector3d::UnitX()) * AngleAxisd(M_PI/4, Vector3d::UnitY());

//    double psi=0;
//    if(!kinematics_->inverseKinematics(xHome3d, quatHome, gc, psi, qHome_)){
//        ROS_INFO_STREAM("No feasible solution!");
//        qHome_ << 0., 0., 0., 0., 0., 0., 0.;
//    }

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
    qHome_ << 1.4667675, 1.1785414, -1.502088, -0.99056375, 0.07174191, 1.7372178, -3.0764043;
    for (int i = 0; i < 7; ++i) {
        jointViaPoint_.positions[i] = qHome_[i];
        jointViaPoint_.velocities[i] = 0.;
    }
    jointViaPoint_.time_from_start = ros::Duration(3.0);
    jointTrajectory_.points.push_back(jointViaPoint_);
    qHome_ << 1.5076244, 1.3209599, -1.4323518, -1.1279348, 0.17179422, 1.6073319, -2.9473362;
    for (int i = 0; i < 7; ++i) {
        jointViaPoint_.positions[i] = qHome_[i];
        jointViaPoint_.velocities[i] = 0.;
    }
    jointViaPoint_.time_from_start = ros::Duration(5.0);
    jointTrajectory_.points.push_back(jointViaPoint_);

    jointTrajectory_.header.stamp = ros::Time::now();
    jointTrajectoryPub_.publish(jointTrajectory_);
    ros::Duration(6.0).sleep();
}

void Agent::update() {
    observationState_ = observer_.getObservation();
    updateTactic();
    if (generateTrajectory()) {
        gotoHome();
    }
}

void Agent::updateTactic() {
    if(observationState_.puckPos.transform.translation.x < 1.2 &&
            observationState_.puckPos.transform.translation.x > 0.7 &&
            observationState_.puckVel.twist.linear.x < 1e-3 &&
            observationState_.puckVel.twist.linear.y < 1e-3){
        tacticState_ = Tactics::SMASH;
    } else{
        tacticState_ = Tactics::CUT;
    }
}

bool Agent::generateTrajectory() {
    if (tacticState_ == Tactics::SMASH) {
        iiwas_kinematics::Kinematics::JointArrayType qCur;
        for (size_t row = 0; row < observationState_.jointState.position.size() - 2; row++) {
            qCur[row] = observationState_.jointState.position[row];
        }
        Vector3d xCur;
        kinematics_->forwardKinematics(qCur, xCur);
        Vector2d xCur2d = xCur.block<2, 1>(0, 0);
        Vector2d puckCur2d(observationState_.puckPos.transform.translation.x,
                           observationState_.puckPos.transform.translation.y);
        double vHitMag = 1.5;

        updateGoal(puckCur2d);
        Vector2d vHit = (xGoal_ - puckCur2d).normalized();
        Vector2d xHit = puckCur2d - vHit * (puckRadius_ + malletRadius_ + 0.005);

        vHit *= vHitMag;

        for (size_t i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            if (!combinatorialHit_ ->plan(xCur2d, xHit, vHit, cartTrajectory_)){
                return false;
            }
//            if (!bezierHit_->plan(xCur2d, xHit, vHit, cartTrajectory_)) {
//                return false;
//            }
            cartTrajectoryPub_.publish(cartTrajectory_);

            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Reduce the desired velocity");
                vHit *= .8;
                continue;
            }
            jointTrajectoryPub_.publish(jointTrajectory_);
            ros::Duration(jointTrajectory_.points.back().time_from_start.toSec() + 1).sleep();
            return true;
        }
        ROS_INFO_STREAM("Failed to find a feasible hitting movement");
        return false;
    }
    return false;
}

double Agent::updateGoal(Vector2d puckPos) {
    if (puckPos(1) > 0.1) {
        xGoal_(1) = -0.1;
    } else if (puckPos(1) < -0.1) {
        xGoal_(1) = 0.1;
    } else {
        xGoal_(1) = 0.0;
    }
    return 0;
}
