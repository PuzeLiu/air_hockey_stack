#include "agent.h"

using namespace AirHockey;

Agent::Agent(ros::NodeHandle nh, double rate) : nh_(nh), rate_(rate), dist_(0, 2),
                                                tfBuffer_(), tfListener_(tfBuffer_) {
    loadParam();

    smashCount_ = 0;

    rng_.seed(0);

    observer_ = new Observer(nh, ros::Rate(rate), defendLine_);
    optimizer_ = new NullSpaceOptimizer(kinematics_, observer_, false);

    combinatorialHit_ = new CombinatorialHit(
            Vector2d(malletRadius_, -tableWidth_ / 2 + malletRadius_ + 0.02),
            Vector2d(tableLength_ - malletRadius_, tableWidth_ / 2 - malletRadius_ - 0.02),
            rate, universalJointHeight_);
    cubicLinearMotion_ = new CubicLinearMotion(rate, universalJointHeight_);

    jointTrajectoryPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
            controllerName_ + "/command", 1);
    cartTrajectoryPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("cartesian_trajectory", 1);

    cartTrajectory_.joint_names.push_back("x");
    cartTrajectory_.joint_names.push_back("y");
    cartTrajectory_.joint_names.push_back("z");

    std::string ns_prefix;
    if (nh.getNamespace() == "/iiwa_front") {
        ns_prefix = 'F';
        tfRobot2Table_ = tfBuffer_.lookupTransform("F_link_0", "TableHome", ros::Time(0), ros::Duration(10.0));
        tfRobot2TableInverse_.header = tfRobot2Table_.header;
        tf2::Stamped<tf2::Transform> tmp;
        tf2::fromMsg(tfRobot2Table_, tmp);
        tfRobot2TableInverse_.transform = tf2::toMsg(tmp.inverse());
    } else if (nh.getNamespace() == "/iiwa_back") {
        ns_prefix = 'B';
        tfRobot2Table_ = tfBuffer_.lookupTransform("B_link_0", "TableAway", ros::Time(0), ros::Duration(10.0));
        tfRobot2TableInverse_.header = tfRobot2Table_.header;
        tf2::Stamped<tf2::Transform> tmp;
        tf2::fromMsg(tfRobot2Table_, tmp);
        tfRobot2TableInverse_.transform = tf2::toMsg(tmp.inverse());
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
    gameStatusPrev_ = GameStatus::STOP;

    Vector3d xTmp, gc;
    Quaterniond quatTmp;
    double psi;
    Kinematics::JointArrayType qInit;
    kinematics_->forwardKinematics(qRef_, xTmp, quatTmp);
    kinematics_->getRedundancy(qRef_, gc, psi);

    xTmp << xHome_[0], xHome_[1], universalJointHeight_ + 0.2;
    applyForwardTransform(xTmp);
    if (!kinematics_->inverseKinematics(xTmp, quatTmp, gc, psi, qInit_)) {
        ROS_ERROR_STREAM("Inverse Kinematics fail, unable to find solution for INIT position");
    }

    xTmp << xHome_[0], xHome_[1], universalJointHeight_;
    applyForwardTransform(xTmp);
    if (!kinematics_->inverseKinematics(xTmp, quatTmp, gc, psi, qHome_)) {
        ROS_ERROR_STREAM("Inverse Kinematics fail, unable to find solution for HOME position");
    }

    trajStopTime_ = ros::Time::now();
}

Agent::~Agent() {
    delete kinematics_;
    delete observer_;
    delete optimizer_;
    delete combinatorialHit_;
    delete cubicLinearMotion_;
}

void Agent::start() {
    ros::Duration(2.).sleep();
    ROS_INFO_STREAM("Agent Start");
    observer_->start();
    ROS_INFO_STREAM("Go to initial position");
    gotoInit(true);
    ros::Duration(5.0).sleep();

    while (ros::ok()) {
        update();
    }
}

void Agent::update() {
    observationState_ = observer_->getObservation();
    bool gameStatusChanged = (observationState_.gameStatus.status != gameStatusPrev_);
    if (gameStatusChanged){
        gameStatusPrev_ = static_cast<GameStatus>(observationState_.gameStatus.status);
        if (observationState_.gameStatus.status == GameStatus::START){
            gotoHome(true);
        } else if (observationState_.gameStatus.status == GameStatus::PAUSE){
            startReady(true);
        } else {
            gotoInit(true);
        }
    } else {
        if (observationState_.gameStatus.status == GameStatus::START) {
            updateTactic();
        }
//        else if (observationState_.gameStatus.status == GameStatus::PAUSE) {
//            startReady(false);
//        } else {
//            gotoInit(false);
//        }
    }
    rate_.sleep();
}

void Agent::updateTactic() {
    if (observationState_.puckPredictedState.state.block<2, 1>(2, 0).norm() > vDefendMin_) {
        if (observationState_.puckPredictedState.predictedTime < maxPredictionTime_ - 1e-6) {
            bool restart = setTactic(Tactics::CUT);
            startCut(restart);
        } else {
            bool restart = setTactic(Tactics::READY);
            startReady(restart);
        }
    } else if (observationState_.puckPredictedState.state.x() < hitRange_[1] &&
               observationState_.puckPredictedState.state.x() > hitRange_[0] &&
               tacticState_ != Tactics::PREPARE) {
        if (observationState_.puckPredictedState.state.y() > tableWidth_ / 2 - puckRadius_ - malletRadius_ - 0.02 ||
            observationState_.puckPredictedState.state.y() <-tableWidth_ / 2 + puckRadius_ + malletRadius_ + 0.02){
            bool restart = setTactic(Tactics::PREPARE);
            startPrepare(restart);
        } else {
            bool restart = setTactic(Tactics::SMASH);
            startHit(restart);
        }
    } else if (observationState_.puckPredictedState.state.x() <= hitRange_[0] ||
               tacticState_ == Tactics::PREPARE) {
        bool restart = setTactic(Tactics::PREPARE);
        startPrepare(restart);
    } else {
        bool restart = setTactic(Tactics::READY);
        startReady(restart);
    }
}

bool Agent::setTactic(Tactics tactic) {
    if (tacticState_ != tactic) {
        if (tactic == Tactics::READY) {
            if (tacticState_ == Tactics::SMASH && ros::Time::now() < trajStopTime_){
                return false;
            } else {
                tacticState_ = tactic;
                ROS_INFO_STREAM("Tactics changed: READY");
                return true;
            }
        }
        else if (tactic == Tactics::CUT) {
            tacticState_ = tactic;
            ROS_INFO_STREAM("Tactics changed: CUT");
            return true;
        }
        else if (tactic == Tactics::PREPARE) {
            tacticState_ = tactic;
            ROS_INFO_STREAM("Tactics changed: PREPARE");
            return true;
        }
        else if (tactic == Tactics::SMASH) {
            tacticState_ = tactic;
            ROS_INFO_STREAM("Tactics changed: SMASH");
            return true;
        }
    }
    return false;
}

void Agent::gotoInit(bool restart) {
    if (restart || ros::Time::now() > trajStopTime_){
        jointTrajectory_.points.clear();
        for (int i = 0; i < 7; ++i) {
            jointViaPoint_.positions[i] = qInit_[i];
            jointViaPoint_.velocities[i] = 0.;
        }
        jointViaPoint_.time_from_start = ros::Duration(5.0);
        jointTrajectory_.points.push_back(jointViaPoint_);

        jointTrajectory_.header.stamp = ros::Time::now();
        jointTrajectoryPub_.publish(jointTrajectory_);
        trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(5.0);
    }
}

void Agent::gotoHome(bool restart) {
    if (restart || ros::Time::now() > trajStopTime_){
        jointTrajectory_.points.clear();
        for (int i = 0; i < 7; ++i) {
            jointViaPoint_.positions[i] = qHome_[i];
            jointViaPoint_.velocities[i] = 0.;
        }
        jointViaPoint_.time_from_start = ros::Duration(2.0);
        jointTrajectory_.points.push_back(jointViaPoint_);

        jointTrajectory_.header.stamp = ros::Time::now();
        jointTrajectoryPub_.publish(jointTrajectory_);
        trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(2.0);
        ros::Duration(2.0).sleep();
    }
}

double Agent::updateGoal(Vector2d puckPosition) {
    auto random_integer = dist_(rng_);
    random_integer = 0;
    if (puckPosition(1) > 0.1) {
        xGoal_.x() = tableLength_;
        xGoal_.y() = -0.1;
    } else if (puckPosition(1) < -0.1) {
        xGoal_.x() = tableLength_;
        xGoal_.y() = 0.1;
    } else {
        xGoal_.x() = tableLength_;
        xGoal_.y() = 0.0;
    }
    if (random_integer == 1) {
        ROS_INFO_STREAM("Strategy: Right");
        xGoal_.y() = -2 * (tableWidth_ / 2 - puckRadius_) - xGoal_.y();
    } else if (random_integer == 2) {
        ROS_INFO_STREAM("Strategy: Left");
        xGoal_.y() = 2 * (tableWidth_ / 2 - puckRadius_) - xGoal_.y();
    } else {
        ROS_INFO_STREAM("Strategy: Middle");
    }
    return 0;
}

void Agent::startHit(bool restart) {
    if (restart || ros::Time::now() > trajStopTime_) {
        ROS_INFO_STREAM("Start Planning! ");
        Vector3d xCur;
        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        applyInverseTransform(xCur);
        Vector2d xCur2d = xCur.block<2, 1>(0, 0);
        Vector2d puckCur2d = observationState_.puckPredictedState.state.block<2, 1>(0, 0);

        updateGoal(puckCur2d);
        Vector2d vHit = (xGoal_ - puckCur2d).normalized();
        Vector2d xHit = puckCur2d - vHit * (puckRadius_ + malletRadius_ + 0.005);

        vHit *= vHitMax_;
        bool success = false;

        for (size_t i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            if (!combinatorialHit_->plan(xCur2d, xHit, vHit, cartTrajectory_)) {
                return;
            }
            transformTrajectory(cartTrajectory_);
            if (optimizer_->optimizeJointTrajectory(cartTrajectory_, observationState_.jointPosition, jointTrajectory_)) {
                success = true;
                break;
            } else {
                vHit *= .8;
                ROS_INFO_STREAM("Optimization Failed. Reduce the velocity: " << vHit.transpose());
                continue;
            }
        }
        //! plan for return trajectory
        trajectory_msgs::MultiDOFJointTrajectory cartTrajReturn;
        trajectory_msgs::JointTrajectory jointTrajReturn;
        if (success){
            success = false;
            double vReadyMax = 0.5;
            cartTrajReturn.joint_names.push_back("x");
            cartTrajReturn.joint_names.push_back("y");
            cartTrajReturn.joint_names.push_back("z");

            for (size_t i = 0; i < 10; ++i) {
                cartTrajReturn.points.clear();
                jointTrajReturn.points.clear();
                cartTrajReturn.points.push_back(cartTrajectory_.points.back());

                Vector3d xStop;
                xStop << cartTrajectory_.points.back().transforms[0].translation.x,
                         cartTrajectory_.points.back().transforms[0].translation.y,
                         cartTrajectory_.points.back().transforms[0].translation.z;
                applyInverseTransform(xStop);
                Vector2d xStop2d = xStop.block<2, 1>(0, 0);
                double tStop = (xHome_ - xStop2d).norm() / vReadyMax;
                cubicLinearMotion_->plan(xStop2d, Vector2d(0., 0.), xHome_, Vector2d(0., 0.), tStop, cartTrajReturn);
                cartTrajReturn.points.erase(cartTrajReturn.points.begin());
                transformTrajectory(cartTrajReturn);

                Kinematics::JointArrayType  qStart;
                for (int j = 0; j < NUM_OF_JOINTS; ++j) {
                    qStart[j] = jointTrajectory_.points.back().positions[j];
                }
//                if (optimizer_->optimizeJointTrajectory(cartTrajReturn, qStart, jointTrajReturn)){
                if (optimizer_->optimizeJointTrajectoryAnchor(cartTrajReturn, qStart, qHome_, jointTrajReturn)){
                    success = true;
                    break;
                } else {
                    vReadyMax *= 0.8;
                    ROS_INFO_STREAM("Optimization Failed. Reduce the velocity for Ready: " << vReadyMax);
                    continue;
                }
            }
        }

        //! append return to whole trajectory
        if (success){
            cartTrajectory_.points.insert(cartTrajectory_.points.end(), cartTrajReturn.points.begin(), cartTrajReturn.points.end());
            jointTrajectory_.points.insert(jointTrajectory_.points.end(), jointTrajReturn.points.begin(), jointTrajReturn.points.end());
            cartTrajectoryPub_.publish(cartTrajectory_);
            jointTrajectory_.header.stamp = ros::Time::now();
            jointTrajectoryPub_.publish(jointTrajectory_);
            trajStopTime_ = ros::Time::now() +
                            ros::Duration(jointTrajectory_.points.back().time_from_start);
            return;
        }
        else{
            ROS_INFO_STREAM("Failed to find a feasible hitting movement");
            setTactic(Tactics::PREPARE);
        }
    }
}

void Agent::startCut(bool restart) {
    Vector2d xCut;
    xCut << defendLine_, observationState_.puckPredictedState.state.y();

    if (restart || ros::Time::now() > trajStopTime_ ||
        (xCut - xCutPrev_).norm() > (puckRadius_ + malletRadius_)) {
        xCutPrev_ = xCut;

        Vector3d xCur, vCur;
        Vector2d xCur2d, vCur2d, vCut;
        Kinematics::JacobianPosType jacobian;

        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        applyInverseTransform(xCur);
        xCur2d = xCur.block<2, 1>(0, 0);
        kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
        vCur = jacobian * observationState_.jointVelocity;
        applyInverseRotation(vCur);
        vCur2d = vCur.block<2, 1>(0, 0);

        double tStop = boost::algorithm::clamp(observationState_.puckPredictedState.predictedTime - 0.3, tDefendMin_,
                                               maxPredictionTime_);
        xCut.y() = boost::algorithm::clamp(xCut.y(), -tableWidth_/2 + malletRadius_ + 0.02, tableWidth_/2 - malletRadius_ - 0.02);

        ROS_INFO_STREAM("Update Plan for CUT");

        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            cubicLinearMotion_->plan(xCur2d, vCur2d, xCut, Vector2d(0., 0.), tStop, cartTrajectory_);
            transformTrajectory(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, observationState_.jointPosition, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Increase the motion time: " << tStop);
                tStop += 0.1;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return;
            }
        }
        cartTrajectoryPub_.publish(cartTrajectory_);
        ROS_INFO_STREAM("Optimization Failed. Unable to find trajectory for Cut");
    }
}

void Agent::startReady(bool restart) {
    if (restart || ros::Time::now() > trajStopTime_) {

        Vector3d xCur, vCur;
        Vector2d xCur2d, vCur2d;
        Kinematics::JacobianPosType jacobian;
        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        applyInverseTransform(xCur);
        xCur2d = xCur.block<2, 1>(0, 0);

        kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
        vCur = jacobian * observationState_.jointVelocity;
        applyInverseRotation(vCur);
        vCur2d = vCur.block<2, 1>(0, 0);

        double tStop = 2.0;
        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();

            cubicLinearMotion_->plan(xCur2d, vCur2d, xHome_, Vector2d(0., 0.), tStop, cartTrajectory_);
            transformTrajectory(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectoryAnchor(cartTrajectory_, observationState_.jointPosition, qHome_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Increase the motion time: " << tStop);
                tStop += 0.1;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return;
            }
        }
        ROS_INFO_STREAM("Optimization Failed. Unable to find trajectory for Ready");
    }
}

void Agent::startPrepare(bool restart) {
    if (restart || ros::Time::now() > trajStopTime_) {
        Vector3d xCur, vCur;
        Vector2d xCur2d, vCur2d, xPuck, xPrepare;
        Kinematics::JacobianPosType jacobian;
        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        applyInverseTransform(xCur);
        xCur2d = xCur.block<2, 1>(0, 0);

        kinematics_->jacobianPos(observationState_.jointPosition, jacobian);
        vCur = jacobian * observationState_.jointVelocity;
        applyInverseRotation(vCur);
        vCur2d = vCur.block<2, 1>(0, 0);

        xPuck = observationState_.puckPredictedState.state.block<2, 1>(0, 0);
//        xPrepare = (xPuck - xCur2d).normalized() * (malletRadius_ + puckRadius_) + xPuck;
        xPrepare = xPuck;
        xPrepare.x() = boost::algorithm::clamp(xPrepare.x(), malletRadius_ + 0.02, tableLength_ - malletRadius_ - 0.02);
        xPrepare.y() = boost::algorithm::clamp(xPrepare.y(),
                                               -tableWidth_ / 2 + malletRadius_ + 0.02,
                                               tableWidth_ / 2 - malletRadius_ - 0.02);


        double tStop = 2.0;
        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();

            cubicLinearMotion_->plan(xCur2d, vCur2d, xPrepare, Vector2d(0., 0.), tStop, cartTrajectory_);
            transformTrajectory(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, observationState_.jointPosition, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Increase the motion time: " << tStop);
                tStop += 0.2;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return;
            }
        }
        ROS_INFO_STREAM("Optimization Failed. Unable to find trajectory for Prepare");
    }
}

void Agent::loadParam() {
    nh_.getParam("/air_hockey/table_length", tableLength_);
    nh_.getParam("/air_hockey/table_width", tableWidth_);
    nh_.getParam("/air_hockey/mallet_radius", malletRadius_);
    nh_.getParam("/air_hockey/puck_radius", puckRadius_);


    nh_.getParam("/air_hockey/agent/universal_joint_height", universalJointHeight_);

    std::vector<double> tcp_position_vec;
    std::vector<double> tcp_quaternion_vec;
    nh_.getParam("/air_hockey/agent/tcp_position", tcp_position_vec);
    nh_.getParam("/air_hockey/agent/tcp_quaternion", tcp_quaternion_vec);
    Vector3d tcp_position_eigen;
    tcp_position_eigen.x() = tcp_position_vec[0];
    tcp_position_eigen.y() = tcp_position_vec[1];
    tcp_position_eigen.z() = tcp_position_vec[2];
    Quaterniond tcp_quaternion_eigen;
    tcp_quaternion_eigen.w() = tcp_quaternion_vec[0];
    tcp_quaternion_eigen.x() = tcp_quaternion_vec[1];
    tcp_quaternion_eigen.y() = tcp_quaternion_vec[2];
    tcp_quaternion_eigen.z() = tcp_quaternion_vec[3];
    kinematics_ = new iiwas_kinematics::Kinematics(tcp_position_eigen, tcp_quaternion_eigen);

    std::vector<double> xTmp;
    nh_.getParam("/air_hockey/agent/home", xTmp);
    xHome_ << xTmp[0], xTmp[1];
    nh_.getParam("/air_hockey/agent/goal", xTmp);
    xGoal_ << xTmp[0], xTmp[1];
    nh_.getParam("/air_hockey/agent/hit_range", xTmp);
    hitRange_ << xTmp[0], xTmp[1];
    nh_.getParam("/air_hockey/agent/q_ref", xTmp);
    qRef_ << xTmp[0], xTmp[1], xTmp[2], xTmp[3], xTmp[4], xTmp[5], xTmp[6];

    nh_.getParam("/air_hockey/agent/defend_line", defendLine_);
    xCutPrev_ << defendLine_, 0.0;

    double frequency;
    int max_prediction_steps;
    nh_.getParam("/air_hockey/puck_tracker/frequency", frequency);
    nh_.getParam("/air_hockey/puck_tracker/max_prediction_steps", max_prediction_steps);

    maxPredictionTime_ = max_prediction_steps / frequency;

    nh_.getParam("/air_hockey/agent/max_hit_velocity", vHitMax_);

    nh_.getParam("/air_hockey/agent/min_defend_velocity", vDefendMin_);

    nh_.getParam("/air_hockey/agent/min_defend_time", tDefendMin_);
    nh_.getParam("/air_hockey/agent/controller", controllerName_);
}

void Agent::transformTrajectory(trajectory_msgs::MultiDOFJointTrajectory &cartesianTrajectory) {
    geometry_msgs::Point tmp;
    for (int i = 0; i < cartesianTrajectory.points.size(); ++i) {
        tmp.x = cartesianTrajectory.points[i].transforms[0].translation.x;
        tmp.y = cartesianTrajectory.points[i].transforms[0].translation.y;
        tmp.z = cartesianTrajectory.points[i].transforms[0].translation.z;

        tf2::doTransform(tmp, tmp, tfRobot2Table_);

        cartesianTrajectory.points[i].transforms[0].translation.x = tmp.x;
        cartesianTrajectory.points[i].transforms[0].translation.y = tmp.y;
        cartesianTrajectory.points[i].transforms[0].translation.z = tmp.z;
    }
}

void Agent::applyForwardTransform(Vector3d &v_in_out) {
    geometry_msgs::Point point;
    point.x = v_in_out.x();
    point.y = v_in_out.y();
    point.z = v_in_out.z();
    tf2::doTransform(point, point, tfRobot2Table_);
    v_in_out.x() = point.x;
    v_in_out.y() = point.y;
    v_in_out.z() = point.z;
}

void Agent::applyInverseTransform(Vector3d &v_in_out) {
    geometry_msgs::Point point;
    point.x = v_in_out.x();
    point.y = v_in_out.y();
    point.z = v_in_out.z();
    tf2::doTransform(point, point, tfRobot2TableInverse_);
    v_in_out.x() = point.x;
    v_in_out.y() = point.y;
    v_in_out.z() = point.z;
}

void Agent::applyInverseRotation(Vector3d &v_in_out) {
    geometry_msgs::Vector3 point;
    point.x = v_in_out.x();
    point.y = v_in_out.y();
    point.z = v_in_out.z();

    tf2::doTransform(point, point, tfRobot2TableInverse_);

    v_in_out.x() = point.x;
    v_in_out.y() = point.y;
    v_in_out.z() = point.z;
}
