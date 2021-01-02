#include "agent.h"

using namespace tactical_agent;

Agent::Agent(ros::NodeHandle nh, double rate) : nh_(nh), rate_(rate), observer_(nh, ros::Rate(rate)) {
    universalJointHeight_ = 0.165;

    kinematics_ = new iiwas_kinematics::Kinematics(Vector3d(0.0, 0.0, 0.515),
                                                   Quaterniond(1.0, 0.0, 0.0, 0.0));
    optimizer_ = new NullSpaceOptimizer(kinematics_, &observer_, false);

    bezierHit_ = new BezierHit(Vector2d(0.58, -0.47),
                               Vector2d(2.43, 0.47),
                               rate,
                               universalJointHeight_);
    combinatorialHit_ = new CombinatorialHit(Vector2d(0.58, -0.45),
                                             Vector2d(2.43, 0.45),
                                             rate,
                                             universalJointHeight_);
    stableDynamicsMotion_ = new StableDynamicsMotion(Vector2d(200.0, 200.0),
                                                  Vector2d(1.0, 1.0),
                                                  rate, universalJointHeight_);

    jointTrajectoryPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
            "joint_position_trajectory_controller/command", 1);
    cartTrajectoryPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cartesian_trajectory", 1);

    tacticState_ = Tactics::READY;
    tacticChanged_ = false;

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

void Agent::gotoInit() {
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

    }
    rate_.sleep();
}

void Agent::updateTactic() {
    if (observationState_.puckVelocity.norm() > 0.01) {
        if (observationState_.puckVelocity.x() < 0) {
            setTactic(Tactics::CUT);
        } else {
            setTactic(Tactics::READY);
        }
    } else if (observationState_.puckPosition.x() < 1.5 &&
               observationState_.puckPosition.x() > 0.7) {
        setTactic(Tactics::SMASH);
    } else if (observationState_.puckPosition.x() <= 0.7) {
        setTactic(Tactics::PREPARE);
    } else {
        setTactic(Tactics::READY);
    }
}

void Agent::setTactic(Tactics tactic) {
    if (tacticState_ != tactic) {
        if (tactic == Tactics::READY) ROS_INFO_STREAM("Tactics changed: READY");
        else if (tactic == Tactics::CUT) ROS_INFO_STREAM("Tactics changed: CUT");
        else if (tactic == Tactics::PREPARE) ROS_INFO_STREAM("Tactics changed: PREPARE");
        else if (tactic == Tactics::SMASH) ROS_INFO_STREAM("Tactics changed: SMASH");
        tacticState_ = tactic;
        tacticChanged_ = true;
    }
    else{
        tacticChanged_ = false;
    }
}

bool Agent::generateTrajectory() {
    if (tacticState_ == Tactics::SMASH) {
        Vector3d xCur;
        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        Vector2d xCur2d = xCur.block<2, 1>(0, 0);
        Vector2d puckCur2d = observationState_.puckPosition.block<2, 1>(0, 0);
        double vHitMag = 1.5;

        updateGoal(puckCur2d);
        Vector2d vHit = (xGoal_ - puckCur2d).normalized();
        Vector2d xHit = puckCur2d - vHit * (puckRadius_ + malletRadius_ + 0.005);

        vHit *= vHitMag;

        for (size_t i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            if (!combinatorialHit_->plan(xCur2d, xHit, vHit, cartTrajectory_)) {
                return false;
            }
//            if (!bezierHit_->plan(xCur2d, xHit, vHit, cartTrajectory_)) {
//                return false;
//            }
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Reduce the desired velocity");
                vHit *= .8;
                continue;
            }
            cartTrajectoryPub_.publish(cartTrajectory_);
            jointTrajectoryPub_.publish(jointTrajectory_);
            ros::Duration(jointTrajectory_.points.back().time_from_start.toSec()).sleep();
            return true;
        }
        ROS_INFO_STREAM("Failed to find a feasible hitting movement");
        tacticState_ = Tactics::PREPARE;
        return false;
    } else if (tacticState_ == Tactics::READY) {
        if (tacticChanged_){
            stableDynamicsMotion_->setStiffness(Vector2d(10.0, 10.0));
        }
        Vector3d xCur, vCur;
        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        Vector2d xCur2d = xCur.block<2, 1>(0, 0);
        Kinematics::JacobianPosType jacobian;
        kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
        vCur = jacobian * observationState_.jointVelocity;
        Vector2d vCur2d = vCur.block<2, 1>(0, 0);

        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            stableDynamicsMotion_->plan(xCur2d, vCur2d, xHome_, cartTrajectory_, 0.1);

            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Reduce the stiffness");
                stableDynamicsMotion_->scaleStiffness(Vector2d(0.8, 0.8));
                continue;
            } else{
                cartTrajectoryPub_.publish(cartTrajectory_);
                jointTrajectoryPub_.publish(jointTrajectory_);
                ros::Duration(jointTrajectory_.points.back().time_from_start.toSec()).sleep();
                stableDynamicsMotion_->scaleStiffness(Vector2d(1.25, 1.25));
                break;
            }
        }
    } else if (tacticState_ == Tactics::CUT) {
        if (tacticChanged_){
            stableDynamicsMotion_->setStiffness(Vector2d(100.0, 100.0));
        }
        Vector3d xCur, vCur;
        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        Vector2d xCur2d = xCur.block<2, 1>(0, 0);
        Kinematics::JacobianPosType jacobian;
        kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
        vCur = jacobian * observationState_.jointVelocity;
        Vector2d vCur2d = vCur.block<2, 1>(0, 0);
        Vector2d xGoal(0.7, observationState_.puckPosition(1));

        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            stableDynamicsMotion_->plan(xCur2d, vCur2d, xGoal, cartTrajectory_, 0.1);

            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Reduce the stiffness");
                stableDynamicsMotion_->scaleStiffness(Vector2d(0.8, 0.8));
                continue;
            } else{
                cartTrajectoryPub_.publish(cartTrajectory_);
                jointTrajectoryPub_.publish(jointTrajectory_);
                ros::Duration(jointTrajectory_.points.back().time_from_start.toSec()).sleep();
                stableDynamicsMotion_->scaleStiffness(Vector2d(1.25, 1.25));
                break;
            }
        }
    }
    return false;
}

double Agent::updateGoal(Vector2d puckPosition) {
    if (puckPosition(1) > 0.1) {
        xGoal_(1) = -0.1;
    } else if (puckPosition(1) < -0.1) {
        xGoal_(1) = 0.1;
    } else {
        xGoal_(1) = 0.0;
    }
    return 0;
}
