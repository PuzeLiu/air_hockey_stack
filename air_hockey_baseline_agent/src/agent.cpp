#include "agent.h"

using namespace AirHockey;

Agent::Agent(ros::NodeHandle nh, double rate) : nh_(nh), rate_(rate), dist_(0, 2),
                                                tfBuffer_(), tfListener_(tfBuffer_) {
    loadParam();

    smashCount_ = 0;
    cutCount_ = 0;
    cutPrevY_ = 0.0;
    staticCount = 0;

    rng_.seed(0);

    observer_ = new Observer(nh, controllerName_, ros::Rate(rate), defendLine_);
    optimizer_ = new NullSpaceOptimizer(kinematics_, observer_, false);

    combinatorialHit_ = new CombinatorialHit(
            Vector2d(malletRadius_, -tableWidth_ / 2 + malletRadius_ + 0.02),
            Vector2d(tableLength_ / 2 - malletRadius_, tableWidth_ / 2 - malletRadius_ - 0.02),
            rate, universalJointHeight_);
    cubicLinearMotion_ = new CubicLinearMotion(rate, universalJointHeight_);

    jointTrajectoryPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
            controllerName_ + "/command", 2);
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

    optimizer_->SolveJoint7(qInit_);
    optimizer_->SolveJoint7(qHome_);

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
    if (gameStatusChanged) {
        gameStatusPrev_ = static_cast<GameStatus>(observationState_.gameStatus.status);
        if (observationState_.gameStatus.status == GameStatus::START) {
            ROS_INFO_STREAM("Game Status Changed: START");
            gotoHome(true);
        } else if (observationState_.gameStatus.status == GameStatus::PAUSE) {
            ROS_INFO_STREAM("Game Status Changed: PAUSE");
            startReady(true);
        } else {
            ROS_INFO_STREAM("Game Status Changed: STOP");
            gotoInit(true);
        }
    } else {
        if (observationState_.gameStatus.status == GameStatus::START) {
            updateTactic();
        }
    }
    rate_.sleep();
}

void Agent::updateTactic() {
    if (observationState_.puckPredictedState.state.block<2, 1>(2, 0).norm() > vDefendMin_){
        staticCount = 0;
    } else {
        staticCount += 1;
    }

    if (staticCount < 10) {
        if (observationState_.puckPredictedState.predictedTime < maxPredictionTime_ - 1e-6) {
            setTactic(Tactics::CUT);
            if (abs(observationState_.puckPredictedState.state.y() - cutPrevY_) > (malletRadius_)) {
                cutCount_ = 0;
                cutPrevY_ = observationState_.puckPredictedState.state.y();
            } else {
                cutCount_ += 1;
            }
            if (cutCount_ == 10) {
                ROS_INFO_STREAM("New Cut Motion");
                startCut(true);
            }
        } else {
            bool restart = setTactic(Tactics::READY);
            startReady(restart);
        }
    } else {
        if (observationState_.puckPredictedState.state.x() < hitRange_[1] &&
            observationState_.puckPredictedState.state.x() > hitRange_[0] &&
            tacticState_ != Tactics::PREPARE) {
            if (observationState_.puckPredictedState.state.y() > tableWidth_ / 2 - puckRadius_ - malletRadius_ - 0.02 ||
                observationState_.puckPredictedState.state.y() < -tableWidth_ / 2 + puckRadius_ + malletRadius_ + 0.02) {
                bool restart = setTactic(Tactics::PREPARE);
//                startPrepare(restart);
//                startPrepareNew(restart);
                startPrepareCombinatorial(restart);
            } else {
                bool restart = setTactic(Tactics::SMASH);
                if (smashCount_ > 10) {
                    startHit(restart);
                }
            }
        } else if (observationState_.puckPredictedState.state.x() <= hitRange_[0] ||
                   tacticState_ == Tactics::PREPARE) {
            bool restart = setTactic(Tactics::PREPARE);
//            startPrepare(restart);
//            startPrepareNew(restart);
            startPrepareCombinatorial(restart);
        } else {
            bool restart = setTactic(Tactics::READY);
            startReady(restart);
        }
    }

}

bool Agent::setTactic(Tactics tactic) {
    if (tacticState_ != tactic) {
        if (tactic == Tactics::READY) {
            if (tacticState_ == Tactics::SMASH && ros::Time::now() < trajStopTime_) {
                return false;
            } else {
                tacticState_ = tactic;
                ROS_INFO_STREAM("Tactics changed: READY");
                return true;
            }
        } else if (tactic == Tactics::CUT) {
            tacticState_ = tactic;
            ROS_INFO_STREAM("Tactics changed: CUT");
            return true;
        } else if (tactic == Tactics::PREPARE) {
            tacticState_ = tactic;
            ROS_INFO_STREAM("Tactics changed: PREPARE");
            return true;
        } else if (tactic == Tactics::SMASH) {
            tacticState_ = tactic;
            smashCount_ = 0;
            ROS_INFO_STREAM("Tactics changed: SMASH");
            return true;
        }
    } else if (tactic == Tactics::SMASH) {
        smashCount_ += 1;
        return false;
    }
    return false;
}

void Agent::gotoInit(bool restart) {
    if (restart || ros::Time::now() > trajStopTime_) {
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
    if (restart || ros::Time::now() > trajStopTime_) {
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
            if (optimizer_->optimizeJointTrajectory(cartTrajectory_, observationState_.jointPosition,
                                                    jointTrajectory_)) {
                success = true;
                break;
            } else {
                vHit *= .8;
                ROS_INFO_STREAM("Optimization Failed [HITTING]. Reduce the velocity: " << vHit.transpose());
                continue;
            }
        }
        //! plan for return trajectory
        trajectory_msgs::MultiDOFJointTrajectory cartTrajReturn;
        trajectory_msgs::JointTrajectory jointTrajReturn;
        double vMax = 0.5;
        success = success && planReturnTraj(vMax, cartTrajectory_.points.back(), cartTrajReturn, jointTrajReturn);

        //! append return to whole trajectory
        if (success) {
            cartTrajectory_.points.insert(cartTrajectory_.points.end(), cartTrajReturn.points.begin(),
                                          cartTrajReturn.points.end());
            jointTrajectory_.points.insert(jointTrajectory_.points.end(), jointTrajReturn.points.begin(),
                                           jointTrajReturn.points.end());
            cartTrajectoryPub_.publish(cartTrajectory_);
            jointTrajectory_.header.stamp = ros::Time::now();
            jointTrajectoryPub_.publish(jointTrajectory_);
            trajStopTime_ = ros::Time::now() +
                            ros::Duration(jointTrajectory_.points.back().time_from_start);
            return;
        } else {
            ROS_INFO_STREAM("Failed to find a feasible movement [HITTING]");
            setTactic(Tactics::PREPARE);
        }
    }
}

void Agent::startCut(bool restart) {
    Vector2d xCut(defendLine_, observationState_.puckPredictedState.state.y());

    if (restart) {
        Vector3d xCur, vCur;
        Vector2d xCur2d, vCur2d;
        ros::Time tStart;
        Kinematics::JointArrayType qStart, dqStart;

        getPlannedState(xCur, vCur, qStart, dqStart, tStart, planTimeOffset_);

        applyInverseTransform(xCur);
        applyInverseRotation(vCur);

        xCur2d = xCur.block<2, 1>(0, 0);
        vCur2d = vCur.block<2, 1>(0, 0);
        double tStop = boost::algorithm::clamp(observationState_.puckPredictedState.predictedTime - 0.3, tDefendMin_,
                                               maxPredictionTime_);
        xCut.y() = boost::algorithm::clamp(xCut.y(), -tableWidth_ / 2 + malletRadius_ + 0.02,
                                           tableWidth_ / 2 - malletRadius_ - 0.02);

        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            cubicLinearMotion_->plan(xCur2d, vCur2d, xCut, Vector2d(0., 0.), tStop, cartTrajectory_);
            transformTrajectory(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, qStart,
//            qAnchor << 0., 0., 0., 0., 0., 0, 0.;
                                                     jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed [Cut]. Increase the motion time: " << tStop);
                tStop += 0.1;
            } else {
                jointTrajectory_.header.stamp = tStart;
                cartTrajectory_.header.stamp = tStart;
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return;
            }
        }
        ROS_INFO_STREAM("Optimization Failed [Cut].");
    }
}

void Agent::startReady(bool restart) {
    if (ros::Time::now() > trajStopTime_) {

        Vector3d xStart, vStart;
        Vector2d xStart2d, vStart2d;
        ros::Time tStart;
        Kinematics::JointArrayType qStart, dqStart;

        getPlannedState(xStart, vStart, qStart, dqStart, tStart, planTimeOffset_);

        applyInverseTransform(xStart);
        applyInverseRotation(vStart);

        xStart2d = xStart.block<2, 1>(0, 0);
        vStart2d = vStart.block<2, 1>(0, 0);

        double tStop = 1.5;
        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            cubicLinearMotion_->plan(xStart2d, vStart2d, xHome_, Vector2d(0., 0.), tStop, cartTrajectory_);
            transformTrajectory(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectoryAnchor(cartTrajectory_, qStart, qHome_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed [READY]. Increase the motion time: " << tStop);
                tStop += 0.1;
            } else {
                jointTrajectory_.header.stamp = tStart;
                cartTrajectory_.header.stamp = tStart;
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return;
            }
        }
        ROS_INFO_STREAM("Optimization Failed [READY]. Unable to find trajectory for Ready");
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
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, observationState_.jointPosition,
                                                     jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed [PREPARE]. Increase the motion time: " << tStop);
                tStop += 0.2;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return;
            }
        }
        ROS_INFO_STREAM("Optimization Failed [PREPARE]. Unable to find trajectory for Prepare");
    }
}

void Agent::startPrepareNew(bool restart) {
    if (ros::Time::now() > trajStopTime_) {
        Vector3d xCur;
        kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
        applyInverseTransform(xCur);

        Vector3d xLiftUp, xSetDown;
        xLiftUp = observationState_.puckPredictedState.state.block<3, 1>(0, 0);
        xLiftUp.x() = boost::algorithm::clamp(xLiftUp.x(), malletRadius_ + 0.02, tableLength_ - malletRadius_ - 0.02);
        xLiftUp.y() = boost::algorithm::clamp(xLiftUp.y(), -tableWidth_ / 2 + malletRadius_ + 0.02,
                                              tableWidth_ / 2 - malletRadius_ - 0.02);
        xLiftUp.z() = prepareHeight_;
        xSetDown = xLiftUp;
        xSetDown.z() = universalJointHeight_ + puckHeight_;

        double tStop = 2.0;
        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();

            cubicLinearMotion_->plan(xCur, Vector3d(0., 0., 0.), xLiftUp, Vector3d(0., 0., 0.), tStop, cartTrajectory_);
            cubicLinearMotion_->plan(xLiftUp, Vector3d(0., 0., 0.), xSetDown, Vector3d(0., 0., 0.), tStop,
                                     cartTrajectory_);
            cubicLinearMotion_->plan(xSetDown, Vector3d(0., 0., 0.), xPrepare_, Vector3d(0., 0., 0.), tStop,
                                     cartTrajectory_);
            transformTrajectory(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, observationState_.jointPosition,
                                                     jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed [PREPARE]. Increase the motion time: " << tStop);
                tStop += 0.2;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(3 * tStop);
                ros::Duration(3 * tStop).sleep();
                return;
            }
        }
        ROS_INFO_STREAM("Optimization Failed [PREPARE]. Unable to find trajectory for Prepare");
    }
}

void Agent::startPrepareCombinatorial(bool restart) {
    if (restart || ros::Time::now() > trajStopTime_) {
        Vector3d xStart, vStart;
        Vector2d xStart2d, vStart2d, xPuck, xPrepare, vPrepare;
        ros::Time tStart;
        Kinematics::JointArrayType qStart, dqStart;

        getPlannedState(xStart, vStart, qStart, dqStart, tStart, planTimeOffset_);
        applyInverseTransform(xStart);
        applyInverseRotation(vStart);

        xStart2d = xStart.block<2, 1>(0, 0);

        xPuck = observationState_.puckPredictedState.state.block<2, 1>(0, 0);
        if (xPuck.y() > 0) {
            vPrepare = (Vector2d(xPuck.x() + 0.2, 2 * (tableWidth_ / 2 - puckRadius_)) - xPuck).normalized();
//            vPrepare = Vector2d(0.1, 0.7);
        } else {
            vPrepare = (Vector2d(xPuck.x() + 0.2, - 2 * (tableWidth_ / 2 - puckRadius_)) - xPuck).normalized();
//            vPrepare = Vector2d(0.1, 0.7);
        }
        xPrepare = xPuck - vPrepare * (puckRadius_ + malletRadius_);
        xPrepare.x() = boost::algorithm::clamp(xPrepare.x(), malletRadius_ + 0.02, tableLength_ - malletRadius_ - 0.02);
        xPrepare.y() = boost::algorithm::clamp(xPrepare.y(),
                                               -tableWidth_ / 2 + malletRadius_ + 0.02,
                                               tableWidth_ / 2 - malletRadius_ - 0.02);
        double tStop = 0.08;
        bool success = false;

        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            combinatorialHit_->plan(xStart2d, xPrepare, vPrepare, cartTrajectory_, tStop);
            transformTrajectory(cartTrajectory_);

            if (optimizer_->optimizeJointTrajectory(cartTrajectory_, observationState_.jointPosition,
                                                    jointTrajectory_)) {
                success = true;
                break;
            } else {
                vPrepare *= .8;
                ROS_INFO_STREAM("Optimization Failed [PREPARE]. Reduce the velocity: " << vPrepare.transpose());
                continue;
            }
        }

        //! plan for return trajectory
        trajectory_msgs::MultiDOFJointTrajectory cartTrajReturn;
        trajectory_msgs::JointTrajectory jointTrajReturn;
        double vMax = 0.5;
        success = success && planReturnTraj(vMax, cartTrajectory_.points.back(), cartTrajReturn, jointTrajReturn);

        //! append return to whole trajectory
        if (success) {
            cartTrajectory_.points.insert(cartTrajectory_.points.end(), cartTrajReturn.points.begin(),
                                          cartTrajReturn.points.end());
            jointTrajectory_.points.insert(jointTrajectory_.points.end(), jointTrajReturn.points.begin(),
                                           jointTrajReturn.points.end());
            cartTrajectory_.header.stamp = tStart;
            jointTrajectory_.header.stamp = tStart;

            cartTrajectoryPub_.publish(cartTrajectory_);
            jointTrajectoryPub_.publish(jointTrajectory_);
            trajStopTime_ = ros::Time::now() +
                            ros::Duration(jointTrajectory_.points.back().time_from_start);
            return;
        } else {
            ROS_INFO_STREAM("Optimization Failed [PREPARE]. Failed to find a feasible hitting movement");
            setTactic(Tactics::PREPARE);
        }
    }
}

void Agent::loadParam() {
    nh_.getParam("/air_hockey/table_length", tableLength_);
    nh_.getParam("/air_hockey/table_width", tableWidth_);
    nh_.getParam("/air_hockey/mallet_radius", malletRadius_);
    nh_.getParam("/air_hockey/puck_radius", puckRadius_);
    nh_.getParam("/air_hockey/puck_height", puckHeight_);

    nh_.getParam("/air_hockey/agent/universal_joint_height", universalJointHeight_);
    nh_.getParam("/air_hockey/agent/prepare_height", prepareHeight_);

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
    nh_.getParam("/air_hockey/agent/prepare", xTmp);
    xPrepare_ << xTmp[0], xTmp[1], universalJointHeight_ + puckHeight_;
    nh_.getParam("/air_hockey/agent/goal", xTmp);
    xGoal_ << xTmp[0], xTmp[1];
    nh_.getParam("/air_hockey/agent/hit_range", xTmp);
    hitRange_ << xTmp[0], xTmp[1];
    nh_.getParam("/air_hockey/agent/q_ref", xTmp);
    qRef_ << xTmp[0], xTmp[1], xTmp[2], xTmp[3], xTmp[4], xTmp[5], xTmp[6];

    nh_.getParam("/air_hockey/agent/defend_line", defendLine_);

    double frequency;
    int max_prediction_steps;
    nh_.getParam("/air_hockey/puck_tracker/frequency", frequency);
    nh_.getParam("/air_hockey/puck_tracker/max_prediction_steps", max_prediction_steps);

    maxPredictionTime_ = max_prediction_steps / frequency;

    nh_.getParam("/air_hockey/agent/max_hit_velocity", vHitMax_);

    nh_.getParam("/air_hockey/agent/min_defend_velocity", vDefendMin_);

    nh_.getParam("/air_hockey/agent/min_defend_time", tDefendMin_);
    nh_.getParam("/air_hockey/agent/plan_time_offset", planTimeOffset_);

    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics)){
        for (int i = 0; i < topics.size(); ++i) {
            if (topics[i].name == nh_.getNamespace() + "/joint_position_trajectory_controller/state"){
                controllerName_ = "joint_position_trajectory_controller";
                break;
            } else if (topics[i].name == nh_.getNamespace() + "/joint_torque_trajectory_controller/state"){
                controllerName_ = "joint_torque_trajectory_controller";
                break;
            }
        }
        if (controllerName_ == ""){
            ROS_ERROR_STREAM("Could not find controller");
        }
    }
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

bool Agent::planReturnTraj(const double &vMax,
                           trajectory_msgs::MultiDOFJointTrajectoryPoint &lastPoint,
                           trajectory_msgs::MultiDOFJointTrajectory &cartTrajReturn,
                           trajectory_msgs::JointTrajectory &jointTrajReturn) {
    double vReadyMax = vMax;
    cartTrajReturn.joint_names.push_back("x");
    cartTrajReturn.joint_names.push_back("y");
    cartTrajReturn.joint_names.push_back("z");

    for (size_t i = 0; i < 10; ++i) {
        cartTrajReturn.points.clear();
        jointTrajReturn.points.clear();
        cartTrajReturn.points.push_back(lastPoint);

        Vector3d xStop;
        xStop << lastPoint.transforms[0].translation.x,
                lastPoint.transforms[0].translation.y,
                lastPoint.transforms[0].translation.z;
        applyInverseTransform(xStop);
        Vector2d xStop2d = xStop.block<2, 1>(0, 0);
        double tStop = (xHome_ - xStop2d).norm() / vReadyMax;
        cubicLinearMotion_->plan(xStop2d, Vector2d(0., 0.), xHome_, Vector2d(0., 0.), tStop, cartTrajReturn);
        cartTrajReturn.points.erase(cartTrajReturn.points.begin());
        transformTrajectory(cartTrajReturn);

        Kinematics::JointArrayType qStart;
        for (int j = 0; j < NUM_OF_JOINTS; ++j) {
            qStart[j] = jointTrajectory_.points.back().positions[j];
        }
        if (optimizer_->optimizeJointTrajectoryAnchor(cartTrajReturn, qStart, qHome_, jointTrajReturn)) {
            return true;
            break;
        } else {
            vReadyMax *= 0.8;
            ROS_INFO_STREAM("Optimization Failed [RETURN]. Reduce the velocity for Ready: " << vReadyMax);
            continue;
        }
    }
    return false;
}

void Agent::getPlannedState(Vector3d &x, Vector3d &dx, Kinematics::JointArrayType &q, Kinematics::JointArrayType &dq,
                            ros::Time &tStart, double offset_t) {
    if (jointTrajectory_.points.size() > 0) {
        tStart = ros::Time::now() + ros::Duration(offset_t);
        ros::Time tLast = jointTrajectory_.header.stamp + jointTrajectory_.points.back().time_from_start;
        if (tStart <= tLast) {
            for (int i = jointTrajectory_.points.size() - 1; i >= 0; --i) {
                if (tStart > jointTrajectory_.header.stamp + jointTrajectory_.points[i].time_from_start) {
                    for (int j = 0; j < NUM_OF_JOINTS; ++j) {
                        q[j] = jointTrajectory_.points[i + 1].positions[j];
                        dq[j] = jointTrajectory_.points[i + 1].velocities[j];
                    }
                    break;
                }
            }
        } else {
            for (int j = 0; j < NUM_OF_JOINTS; ++j) {
                q[j] = jointTrajectory_.points.back().positions[j];
                dq[j] = jointTrajectory_.points.back().velocities[j];
            }
        }
    } else {
        q = observationState_.jointPosition;
        dq = observationState_.jointVelocity;
    }
    kinematics_->forwardKinematics(q, x);
    Kinematics::JacobianPosType jacobian;
    kinematics_->jacobianPos(q, jacobian);
    dx = jacobian * dq;
}
