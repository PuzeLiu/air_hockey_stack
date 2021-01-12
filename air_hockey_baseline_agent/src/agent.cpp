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
            Vector2d(malletRadius_, -tableWidth_ / 2 + malletRadius_),
            Vector2d(tableLength_ - malletRadius_, tableWidth_ / 2 - malletRadius_),
            rate,
            universalJointHeight_);
    cubicLinearMotion_ = new CubicLinearMotion(rate, universalJointHeight_);

    jointTrajectoryPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
            "joint_position_trajectory_controller/command", 1);
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
    observationState_.gameStatus.status = 0;           //! Initial game status wit STOP;
    tacticChanged_ = true;

    trajStopTime_ = ros::Time::now();
}

Agent::~Agent() {
    delete kinematics_;
    delete observer_;
    delete optimizer_;
    delete combinatorialHit_;
    delete cubicLinearMotion_;
}

void Agent::gotoInit() {
    jointTrajectory_.points.clear();
    qHome_ << 1.4667675, 1.1785414, -1.502088, -0.99056375, 0.07174191, 1.7372178, 0.;
    for (int i = 0; i < 7; ++i) {
        jointViaPoint_.positions[i] = qHome_[i];
        jointViaPoint_.velocities[i] = 0.;
    }
    jointViaPoint_.time_from_start = ros::Duration(3.0);
    jointTrajectory_.points.push_back(jointViaPoint_);
    qHome_ << 1.5076244, 1.3209599, -1.4323518, -1.1279348, 0.17179422, 1.6073319, 0.;
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
    observationState_ = observer_->getObservation();
    if (observationState_.gameStatus.status != 2) {
        //! If game is not START
        setTactic(Tactics::READY);
    } else {
        if (observationState_.puckPredictedState.state.block<2, 1>(2, 0).norm() > vDefendMin_) {
            if (observationState_.puckPredictedState.predictedTime < maxPredictionTime_ - 1e-6) {
                setTactic(Tactics::CUT);
            } else {
                setTactic(Tactics::READY);
            }
        } else if (observationState_.puckPredictedState.state.x() < hitRange_[1] &&
                   observationState_.puckPredictedState.state.x() > hitRange_[0] &&
                   tacticState_ != Tactics::PREPARE) {
            if (tacticState_ == Tactics::SMASH) { smashCount_ += 1; }
            else { smashCount_ = 0; }
            setTactic(Tactics::SMASH);
        } else if (observationState_.puckPredictedState.state.x() <= hitRange_[0] ||
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

void Agent::start() {
    ROS_INFO_STREAM("Go to home position");
    ros::Duration(2.).sleep();
    gotoInit();
    ROS_INFO_STREAM("Start");
    observer_->start();

    while (ros::ok()) {
        update();
        rate_.sleep();
    }
}

bool Agent::startHit() {
    observationState_ = observer_->getObservation();

    Vector3d xCur;
    kinematics_->forwardKinematics(observationState_.jointPosition, xCur);
    applyInverseTransform(xCur);
    Vector2d xCur2d = xCur.block<2, 1>(0, 0);
    Vector2d puckCur2d = observationState_.puckPredictedState.state.block<2, 1>(0, 0);

    updateGoal(puckCur2d);
    Vector2d vHit = (xGoal_ - puckCur2d).normalized();
    Vector2d xHit = puckCur2d - vHit * (puckRadius_ + malletRadius_ + 0.005);

    vHit *= vHitMax_;

    for (size_t i = 0; i < 10; ++i) {
        cartTrajectory_.points.clear();
        jointTrajectory_.points.clear();
        if (!combinatorialHit_->plan(xCur2d, xHit, vHit, cartTrajectory_)) {
            return false;
        }
        applyTransform(cartTrajectory_);
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
    observationState_ = observer_->getObservation();
    Vector2d xCut;
    xCut << defendLine_, observationState_.puckPredictedState.state.y();

    if (tacticChanged_ || ros::Time::now() > trajStopTime_ ||
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

        ROS_INFO_STREAM("Update Plan for CUT");

        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();
            cubicLinearMotion_->plan(xCur2d, vCur2d, xCut, Vector2d(0., 0.), tStop, cartTrajectory_);
            applyTransform(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Increase the motion time: " << tStop);
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
}

bool Agent::startReady() {
    if (tacticChanged_ || ros::Time::now() > trajStopTime_) {
        observationState_ = observer_->getObservation();

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

        double tStop = 0.8;
        for (int i = 0; i < 10; ++i) {
            cartTrajectory_.points.clear();
            jointTrajectory_.points.clear();

            cubicLinearMotion_->plan(xCur2d, vCur2d, xHome_, Vector2d(0., 0.), tStop, cartTrajectory_);
            applyTransform(cartTrajectory_);
            if (!optimizer_->optimizeJointTrajectory(cartTrajectory_, jointTrajectory_)) {
                ROS_INFO_STREAM("Optimization Failed. Increase the motion time: " << tStop);
                tStop += 0.1;
            } else {
                jointTrajectory_.header.stamp = ros::Time::now();
                jointTrajectoryPub_.publish(jointTrajectory_);
                cartTrajectoryPub_.publish(cartTrajectory_);
                trajStopTime_ = jointTrajectory_.header.stamp + ros::Duration(tStop);
                return true;
            }
        }
        ROS_INFO_STREAM("Optimization Failed. Unable to find trajectory for Ready");
        return false;
    }
    return false;
}

bool Agent::startPrepare() {
    return false;
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

    Vector2d tablePos;
    tablePos << 1.504, 0;

    std::vector<double> xTmp;
    nh_.getParam("/air_hockey/agent/home", xTmp);
    xHome_ << xTmp[0], xTmp[1];
    nh_.getParam("/air_hockey/agent/goal", xTmp);
    xGoal_ << xTmp[0], xTmp[1];
    nh_.getParam("/air_hockey/agent/hit_range", xTmp);
    hitRange_ << xTmp[0], xTmp[1];

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
}

void Agent::applyTransform(trajectory_msgs::MultiDOFJointTrajectory &cartesianTrajectory) {
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
