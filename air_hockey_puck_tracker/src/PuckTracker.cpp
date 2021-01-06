/*
 * MIT License
 * Copyright (c) 2021 Puze Liu, Davide Tateo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "air_hockey_puck_tracker/PuckTracker.hpp"

using namespace AirHockey;

PuckTracker::PuckTracker(ros::NodeHandle nh, ros::Rate rate) : nh_(nh), rate_(rate), tfBuffer_(),
                                                               tfListener_(tfBuffer_, nh_) {
    tfBuffer_.setUsingDedicatedThread(true);
    init();
}

PuckTracker::~PuckTracker() {
    if (thread_.joinable()) {
        thread_.join();
    }
    delete kalmanFilter_;
    delete puckPredictor_;
    delete systemModel_;
    delete observationModel_;
    delete collisionModel_;
}

void PuckTracker::start() {
    thread_ = boost::thread(&PuckTracker::startTracking, this);
}

const PuckPredictedState& PuckTracker::getPredictedState() {
    getPrediction();
    predictedState_.state = puckPredictor_->getState();
    puckPredictor_->getCovarianceSquareRoot();
    predictedState_.time = predictedTime_;
    return predictedState_;
}

void PuckTracker::init() {
    ROS_INFO_STREAM("Namepsace: " << nh_.getNamespace());

    ROS_INFO_STREAM("Read System Parameters");
    double frictionDrag, frictionSliding, restitutionTable, restitutionMallet;
    nh_.param<double>("/puck_tracker/friction_drag", frictionDrag, 0.1);
    nh_.param<double>("/puck_tracker/friction_sliding", frictionSliding, 0.0);
    nh_.param<double>("/puck_tracker/restitution_table", restitutionTable, 0.8);
    nh_.param<double>("/puck_tracker/restitution_mallet", restitutionMallet, 0.1);
    nh_.param<int>("/puck_tracker/max_prediction_steps", maxPredictionSteps_, 20);

    ROS_INFO_STREAM("Drag Parameter:" << frictionDrag);
    ROS_INFO_STREAM("Sliding Parameter: " << frictionSliding);
    ROS_INFO_STREAM("Restitution Parameter of Wall: " << restitutionTable);
    ROS_INFO_STREAM("Restitution Parameter of Mallet: " << restitutionMallet);
    ROS_INFO_STREAM("Prediction Steps: " << maxPredictionSteps_);

    if (nh_.getNamespace() == "/iiwa_front") { robotBaseName_ = "F_link_0";}
    else if (nh_.getNamespace() == "/iiwa_back") { robotBaseName_ = "B_link_0";}
    else {
        ROS_ERROR_STREAM("node should run in namespace: /iiwa_front or /iiwa_back");
        nh_.shutdown();
    }
    ROS_INFO_STREAM("Wait for transform: /Table");
    ros::Duration(1.0).sleep();
    try {
        tfTableStatic_ = tfBuffer_.lookupTransform(robotBaseName_, "Table", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Could not transform " << robotBaseName_ << " to Table: " << exception.what());
        nh_.shutdown();
    }

    systemModel_ = new SystemModel(frictionDrag, frictionSliding);
    observationModel_ = new ObservationModel;
    collisionModel_ = new CollisionModel(tfTableStatic_, restitutionTable, restitutionMallet,
                                         rate_.expectedCycleTime().toSec());
    kalmanFilter_ = new EKF_Wrapper;
    puckPredictor_ = new EKF_Wrapper;

    //! Initialize Kalman Filter
    State sInit;
    sInit.setZero();
    kalmanFilter_->init(sInit);
    setCovariance();

    u_.dt() = rate_.expectedCycleTime().toSec();
}

void PuckTracker::setCovariance() {
    Kalman::Covariance<State> covDyn;
    Kalman::Covariance<Measurement> covObs;
    Kalman::Covariance<State> covInit;

    covObs.setIdentity();
    covDyn.setIdentity();
    covInit.setIdentity();

    double obsVarPos, obsVarAng, dynVarPos, dynVarVel, dynVarAngPos, dynVarAngVel;
    nh_.param<double>("/puck_tracker/observation_variance_position", obsVarPos, 1e-6);
    nh_.param<double>("/puck_tracker/observation_variance_angular", obsVarAng, 1e-6);
    nh_.param<double>("/puck_tracker/dynamic_variance_position", dynVarPos, 1e-4);
    nh_.param<double>("/puck_tracker/dynamic_variance_velocity", dynVarVel, 1e-2);
    nh_.param<double>("/puck_tracker/dynamic_variance_angular_position", dynVarAngPos, 1e-4);
    nh_.param<double>("/puck_tracker/dynamic_variance_angular_velocity", dynVarAngVel, 1e-2);
    ROS_INFO_STREAM("Observer Variance Position: " << obsVarPos);
    ROS_INFO_STREAM("Observer Variance Angular Position: " << obsVarAng);
    ROS_INFO_STREAM("Dynamics Variance Position:" << dynVarPos);
    ROS_INFO_STREAM("Dynamics Variance Velocity: " << dynVarVel);
    ROS_INFO_STREAM("Dynamics Variance Angular Position:" << dynVarAngPos);
    ROS_INFO_STREAM("Dynamics Variance Angular Velocity: " << dynVarAngVel);

    covDyn(0, 0) = dynVarPos;
    covDyn(1, 1) = dynVarPos;
    covDyn(2, 2) = dynVarVel;
    covDyn(3, 3) = dynVarVel;
    covDyn(4, 4) = dynVarAngPos;
    covDyn(5, 5) = dynVarAngVel;

    covObs(0, 0) = obsVarPos;
    covObs(1, 1) = obsVarPos;
    covObs(2, 2) = obsVarAng;

    covInit(0, 0) = 0.5;
    covInit(1, 1) = 0.5;
    covInit(2, 2) = 1.0;
    covInit(3, 3) = 1.0;
    covInit(4, 4) = 0.5;
    covInit(5, 5) = 1.0;

    systemModel_->setCovariance(covDyn);
    observationModel_->setCovariance(covObs);
    kalmanFilter_->setCovariance(covInit);
}

void PuckTracker::startTracking() {
    try {
        tfPuck_ = tfBuffer_.lookupTransform(robotBaseName_, "Puck", ros::Time::now(), ros::Duration(1.0));
        stamp_ = tfPuck_.header.stamp;
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Could not transform " << robotBaseName_ << " to Puck: " << exception.what());
        nh_.shutdown();
    }

    while (nh_.ok()) {
        //! predict step
        kalmanFilter_->predict(*systemModel_, u_);
        collisionModel_->applyCollision(kalmanFilter_->getState(), false);
        ROS_INFO_STREAM("angular velocity: " << kalmanFilter_->getState().dtheta() );
        try {
            //! Update step
            stamp_ += rate_.expectedCycleTime();
            rate_.sleep();
            tfPuck_ = tfBuffer_.lookupTransform(robotBaseName_, "Puck", stamp_, rate_.expectedCycleTime());

            measurement_.x() = tfPuck_.transform.translation.x;
            measurement_.y() = tfPuck_.transform.translation.y;
            tf2::Quaternion quat;
            quat.setX(tfPuck_.transform.rotation.x);
            quat.setY(tfPuck_.transform.rotation.y);
            quat.setZ(tfPuck_.transform.rotation.z);
            quat.setW(tfPuck_.transform.rotation.w);
            tf2::Matrix3x3 rotMat(quat);
            double roll, pitch, yaw;
            rotMat.getEulerYPR(yaw, pitch, roll);
            measurement_.theta() = yaw;
            kalmanFilter_->update(*observationModel_, measurement_);
        } catch (tf2::TransformException &exception){
        }
    }
}

void PuckTracker::getPrediction() {
    //! predict future steps
    puckPredictor_->init(kalmanFilter_->getState());
    puckPredictor_->setCovariance(kalmanFilter_->getCovariance());
    for (int i = 0; i < maxPredictionSteps_; ++i) {
        puckPredictor_->predict(*systemModel_, u_);
        collisionModel_->applyCollision(puckPredictor_->getState(), false);
        predictedTime_ = i * rate_.expectedCycleTime().toSec();
    }
}
