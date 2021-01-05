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
                                                               tfListener_(tfBuffer_) {
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

const PuckPredictedState& PuckTracker::getPuckState() {
    predictionMutex_.lock();
    predictedState_.state = puckPredictor_->getState();
    predictedState_.time = predictedTime_;
    predictionMutex_.unlock();
}

void PuckTracker::init() {
    ROS_INFO_STREAM("Namepsace: " << nh_.getNamespace());

    ROS_INFO_STREAM("Read System Parameters");

    double frictionDrag, frictionSliding, restitutionTable, restitutionMallet;
    nh_.param<double>("friction_drag", frictionDrag, 0.1);
    nh_.param<double>("friction_sliding", frictionSliding, 0.0);
    nh_.param<double>("restitution_table", restitutionTable, 0.8);
    nh_.param<double>("restitution_mallet", restitutionMallet, 0.1);
    nh_.param<int>("max_prediction_steps", maxPredictionSteps_, 20);

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
    try {
        tfTableStatic_ = tfBuffer_.lookupTransform(robotBaseName_, "Table", ros::Time(0), ros::Duration(5.0));
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
}

void PuckTracker::setCovariance() {
    Kalman::Covariance<State> covDyn;
    Kalman::Covariance<Measurement> covObs;
    Kalman::Covariance<State> covInit;

    covObs.setIdentity();
    covDyn.setIdentity();
    covInit.setIdentity();

    double obsVarPos, obsVarAng, dynVarPos, dynVarVel, dynVarAngPos, dynVarAngVel;
    nh_.param<double>("observation_variance_position", obsVarPos, 1e-6);
    nh_.param<double>("observation_variance_angular", obsVarAng, 1e-6);
    nh_.param<double>("dynamic_variance_position", dynVarPos, 1e-4);
    nh_.param<double>("dynamic_variance_velocity", dynVarVel, 1e-2);
    nh_.param<double>("dynamic_variance_angular_position", dynVarAngPos, 1e-4);
    nh_.param<double>("dynamic_variance_angular_velocity", dynVarAngVel, 1e-2);
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
    ROS_INFO_STREAM("Wait for transform: /Puck");
    try {
        tfPuck_ = tfBuffer_.lookupTransform(robotBaseName_, "Table", ros::Time(0), ros::Duration(5.0));
        tfPuckPrev_ = tfPuck_;
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Could not transform " << robotBaseName_ << " to Table: " << exception.what());
        nh_.shutdown();
    }

    while (nh_.ok()) {
        //! predict step
        u_.dt() = rate_.cycleTime().toSec();

        kalmanFilter_->predict(*systemModel_, u_);
        collisionModel_->m_table.applyCollision(kalmanFilter_->getState());

        //! predict future step
        puckPredictor_->init(kalmanFilter_->getState());
        puckPredictor_->setCovariance(kalmanFilter_->getCovariance());
        u_.dt() = rate_.expectedCycleTime().toSec();
        predictionMutex_.lock();
        for (int i = 0; i < maxPredictionSteps_; ++i) {
            puckPredictor_->predict(*systemModel_, u_);
            collisionModel_->m_table.applyCollision(puckPredictor_->getState());
            predictedTime_ = i * rate_.expectedCycleTime().toSec();
        }
        predictionMutex_.unlock();

        //! Update step
        try {
            tfPuck_ = tfBuffer_.lookupTransform(robotBaseName_, "Puck", ros::Time(0));

            if (tfPuck_.header.stamp > tfPuckPrev_.header.stamp){
                measurement_.x() = tfPuck_.transform.translation.x;
                measurement_.y() = tfPuck_.transform.translation.y;
                Eigen::Quaterniond quat;
                quat.x() = tfPuck_.transform.rotation.x;
                quat.y() = tfPuck_.transform.rotation.y;
                quat.z() = tfPuck_.transform.rotation.z;
                quat.w() = tfPuck_.transform.rotation.w;
                measurement_.theta() = quat.toRotationMatrix().eulerAngles(0, 1, 2)(2);
                kalmanFilter_->update(*observationModel_, measurement_);
                tfPuckPrev_ = tfPuck_;
            }
        } catch (tf2::TransformException &exception){
            ROS_ERROR_STREAM("Could not transform " << robotBaseName_ << " to Puck: " << exception.what());
        }

        rate_.sleep();
    }
}
