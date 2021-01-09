#include "agent.h"

using namespace AirHockey;

Agent::Agent(ros::NodeHandle nh, double rate) : nh_(nh), rate_(rate), observer_(nh, ros::Rate(rate)), dist_(0, 2) {
    universalJointHeight_ = 0.165;
    puckRadius_ = 0.03165;
    malletRadius_ = 0.04815;
    Vector2d tablePos;
    tablePos << 1.504, 0;
    tableEdge_ << tablePos.x() - 0.98, tablePos.x() + 0.98,
            tablePos.y() - 0.52, tablePos.y() + 0.52;
    smashCount_ = 0;

    rng_.seed(0);

    kinematics_ = new iiwas_kinematics::Kinematics(Vector3d(0.0, 0.0, 0.515),
                                                   Quaterniond(1.0, 0.0, 0.0, 0.0));
    optimizer_ = new NullSpaceOptimizer(kinematics_, &observer_, false);

    combinatorialHit_ = new CombinatorialHit(
            Vector2d(tableEdge_(0, 0) + malletRadius_, tableEdge_(1, 0) + malletRadius_),
            Vector2d(tableEdge_(0, 1) - malletRadius_, tableEdge_(1, 1) - malletRadius_),
            rate,
            universalJointHeight_);
    stableDynamicsMotion_ = new StableDynamicsMotion(Vector2d(300.0, 300.0),
                                                     Vector2d(1.0, 1.0),
                                                     rate, universalJointHeight_);
    cubicLinearMotion_ = new CubicLinearMotion(rate, universalJointHeight_);

    jointTrajectoryPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
            "joint_position_trajectory_controller/command", 1);
    cartTrajectoryPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cartesian_trajectory", 1);


    xHome_ << 0.58, 0.0;
    xGoal_ << 2.484, 0.0;
    xCutPrev_ << 0.7, 0.0;

    cartTrajectory_.joint_names.push_back("x");
    cartTrajectory_.joint_names.push_back("y");
    cartTrajectory_.joint_names.push_back("z");

    qHome_ << 1.4568405, 1.410468, -1.6053885, -1.0593628, 0.34509358, 1.6828097, 0.;
    Vector3d xRef, gc;
    Quaterniond quatHome;
    double psi;
    kinematics_->forwardKinematics(qHome_, xRef, quatHome);
    kinematics_->getRedundancy(qHome_, gc, psi);
    xRef << xHome_[0], xHome_[1], universalJointHeight_;
    if (!kinematics_->inverseKinematics(xRef, quatHome, gc, psi, qHome_)) {
        ROS_INFO_STREAM("No feasible solution!");
    }

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

    tacticState_ = Tactics::READY;
    observationState_.gameStatus = 0;           //! Initial game status wit STOP;
    tacticChanged_ = true;

    double frequency;
    int max_prediction_steps;
    nh_.param<double>("/puck_tracker/frequency", frequency, 120.);
    nh_.param<int>("/puck_tracker/max_prediction_steps", max_prediction_steps, 20);

    maxPredictionTime_ = max_prediction_steps / frequency;
    ROS_INFO_STREAM("Max Prediction Time: " << maxPredictionTime_);
    trajStopTime_ = ros::Time::now();
}

Agent::~Agent() {
    delete kinematics_;
    delete optimizer_;
    delete combinatorialHit_;
    delete stableDynamicsMotion_;
    delete cubicLinearMotion_;
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
    updateTactic();
    if (generateTrajectory()) {
    }
    rate_.sleep();
}

void Agent::updateTactic() {
    observationState_ = observer_.getObservation();
    if (observationState_.gameStatus != 1) {
        //! If game is not START
        setTactic(Tactics::READY);
    } else {
        if (observationState_.puckPredictedState.state.block<2, 1>(2, 0).norm() > 0.01) {
            if (observationState_.puckPredictedState.predictedTime < maxPredictionTime_ - 1e-6) {
                setTactic(Tactics::CUT);
            } else {
                setTactic(Tactics::READY);
            }
        } else if (observationState_.puckPredictedState.state.x() < 1.5 &&
                   observationState_.puckPredictedState.state.x() > 0.7 &&
                   tacticState_ != Tactics::PREPARE) {
            if (tacticState_ == Tactics::SMASH) { smashCount_ += 1; }
            else { smashCount_ = 0; }
            setTactic(Tactics::SMASH);
        } else if (observationState_.puckPredictedState.state.x() <= 0.7 ||
                   tacticState_ == Tactics::PREPARE) {
            setTactic(Tactics::PREPARE);
        } else {
            setTactic(Tactics::READY);
        }
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
    } else {
        tacticChanged_ = false;
    }
}

bool Agent::generateTrajectory() {
    if (tacticState_ == Tactics::SMASH and smashCount_ > 10) {
        return startHit();
    } else if (tacticState_ == Tactics::READY) {
        return startReady();
    } else if (tacticState_ == Tactics::CUT) {
        return startCut();
    } else {
        return startPrepare();
    }
}

double Agent::updateGoal(Vector2d puckPosition) {
    auto random_integer = dist_(rng_);
//    random_integer = 1;
    if (puckPosition(1) > 0.1) {
        xGoal_.x() = tableEdge_(0, 1);
        xGoal_.y() = -0.1;
    } else if (puckPosition(1) < -0.1) {
        xGoal_.x() = tableEdge_(0, 1);
        xGoal_.y() = 0.1;
    } else {
        xGoal_.x() = tableEdge_(0, 1);
        xGoal_.y() = 0.0;
    }
    if (random_integer == 1) {
        ROS_INFO_STREAM("Strategy: Right");
        xGoal_.y() = 2 * (tableEdge_(1, 0) + puckRadius_) - xGoal_.y();
    } else if (random_integer == 2) {
        ROS_INFO_STREAM("Strategy: Left");
        xGoal_.y() = 2 * (tableEdge_(1, 1) - puckRadius_) - xGoal_.y();
    } else {
        ROS_INFO_STREAM("Strategy: Middle");
    }
    return 0;
}

void Agent::start() {
    ROS_INFO_STREAM("Go to home position");
    ros::Duration(2.).sleep();
    gotoInit();
    ROS_INFO_STREAM("Start");
    observer_.start();

    while (ros::ok()) {
        update();
        rate_.sleep();
    }
}

bool Agent::startHit() {
    observationState_ = observer_.getObservation();

    Vector3d xCur;
    kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
    Vector2d xCur2d = xCur.block<2, 1>(0, 0);
    Vector2d puckCur2d = observationState_.puckPredictedState.state.block<2, 1>(0, 0);
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
        if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
            ROS_INFO_STREAM("Optimization Failed. Reduce the desired velocity");
            vHit *= .8;
            continue;
        }
        cartTrajectoryPub_.publish(cartTrajectory_);
        jointTrajectory_.header.stamp = ros::Time::now();
        jointTrajectoryPub_.publish(jointTrajectory_);
        ros::Duration(jointTrajectory_.points.back().time_from_start.toSec()).sleep();
        return true;
    }
    ROS_INFO_STREAM("Failed to find a feasible hitting movement");
    setTactic(Tactics::PREPARE);
    return false;
}

bool Agent::startCut() {
    observationState_ = observer_.getObservation();
    Vector2d xCut;
    xCut << 0.7, observationState_.puckPredictedState.state.y();

    if (tacticChanged_ || ros::Time::now() > trajStopTime_ || (xCut - xCutPrev_).norm() > puckRadius_) {
        xCutPrev_ = xCut;

        Vector3d xCur, vCur;
        Vector2d xCur2d, vCur2d, vCut;
        Kinematics::JacobianPosType jacobian;

        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        xCur2d = xCur.block<2, 1>(0, 0);
        kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
        vCur = jacobian * observationState_.jointVelocity;
        vCur2d = vCur.block<2, 1>(0, 0);

        double tStop = boost::algorithm::clamp(observationState_.puckPredictedState.predictedTime - 0.3, 0.3, maxPredictionTime_);

        ROS_INFO_STREAM("Update Plan for CUT");

        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            cubicLinearMotion_->plan(xCur2d, vCur2d, xCut,Vector2d(0., 0.), tStop, cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                tStop += 0.1;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return true;
            }
        }
        cartTrajectoryPub_.publish(cartTrajectory_);
        ROS_INFO_STREAM("Optimization Failed. Unable to find trajectory for Cut");
        return false;
    }
    return false;




//    if (tacticChanged_) {
//        stableDynamicsMotion_->setStiffness(Vector2d(200.0, 200.0));
//    }
//    Vector3d xCur, vCur;
//    kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
//    Vector2d xCur2d = xCur.block<2, 1>(0, 0);
//    Kinematics::JacobianPosType jacobian;
//    kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
//    vCur = jacobian * observationState_.jointVelocity;
//    Vector2d vCur2d = vCur.block<2, 1>(0, 0);
//    xGoal_ << 0.7, observationState_.puckPredictedState.state.y();
//    for (int i = 0; i < 10; ++i) {
//        cartTrajectory_.points.clear();
//        jointTrajectory_.points.clear();
//        stableDynamicsMotion_->plan(xCur2d, vCur2d, xGoal_, cartTrajectory_, 0.1);
//
//        if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
//            ROS_INFO_STREAM(
//                    "Optimization Failed. Reduce the CUT stiffness: "
//                            << stableDynamicsMotion_->getStiffness().transpose());
//            stableDynamicsMotion_->scaleStiffness(Vector2d(0.8, 0.8));
//            continue;
//        } else {
//            jointTrajectory_.header.stamp = ros::Time::now();
//            jointTrajectoryPub_.publish(jointTrajectory_);
//            cartTrajectoryPub_.publish(cartTrajectory_);
//            ros::Duration(jointTrajectory_.points.back().time_from_start.toSec()).sleep();
//            break;
//        }
//    }
}

bool Agent::startReady() {
    if (tacticChanged_ || ros::Time::now() > trajStopTime_) {
        Vector3d xCur, vCur;
        Vector2d xCur2d, vCur2d;
        Kinematics::JacobianPosType jacobian;
        double tStop = 0.8;
        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            observationState_ = observer_.getObservation();
            kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
            xCur2d = xCur.block<2, 1>(0, 0);

            kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
            vCur = jacobian * observationState_.jointVelocity;
            vCur2d = vCur.block<2, 1>(0, 0);

            cubicLinearMotion_->plan(xCur2d, vCur2d, xHome_,Vector2d(0., 0.), tStop, cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                tStop += 0.1;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return true;
            }
        }
        cartTrajectoryPub_.publish(cartTrajectory_);
        ROS_INFO_STREAM("Optimization Failed. Unable to find trajectory for Ready");
        return false;
    }
    return false;


//    if (tacticChanged_) {
//        ROS_INFO_STREAM("Reset stiffness READY");
//        stableDynamicsMotion_->setStiffness(Vector2d(200.0, 200.0));
//    }
//    Vector3d xCur, vCur;
//    kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
//    Vector2d xCur2d = xCur.block<2, 1>(0, 0);
//    Kinematics::JacobianPosType jacobian;
//    kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
//    vCur = jacobian * observationState_.jointVelocity;
//    Vector2d vCur2d = vCur.block<2, 1>(0, 0);
//
//    for (int i = 0; i < 10; ++i) {
//        cartTrajectory_.points.clear();
//        jointTrajectory_.points.clear();
//        stableDynamicsMotion_->plan(xCur2d, vCur2d, xHome_, cartTrajectory_, 0.1);
//
//        if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
//            ROS_INFO_STREAM("Optimization Failed. Reduce the READY stiffness: "
//                                    << stableDynamicsMotion_->getStiffness().transpose());
//            stableDynamicsMotion_->scaleStiffness(Vector2d(0.8, 0.8));
//            continue;
//        } else {
//            jointTrajectory_.header.stamp = ros::Time::now();
//            jointTrajectoryPub_.publish(jointTrajectory_);
//            cartTrajectoryPub_.publish(cartTrajectory_);
////                stableDynamicsMotion_->scaleStiffness(Vector2d(1.25, 1.25));
//            ros::Duration(jointTrajectory_.points.back().time_from_start.toSec()).sleep();
////                rate_.sleep();
//            break;
//        }
//    }
}

bool Agent::startPrepare() {
    return false;
}