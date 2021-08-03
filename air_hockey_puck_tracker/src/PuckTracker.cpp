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

using namespace air_hockey_baseline_agent;

PuckTracker::PuckTracker(ros::NodeHandle nh, double defendLine) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {
    tfBuffer_.setUsingDedicatedThread(true);
    defendingLine_ = defendLine;
    loadParams();
}

PuckTracker::~PuckTracker() {
    if (thread_.joinable()) {
        thread_.join();
    }
    if (thread1_.joinable()) {
        thread1_.join();
    }
    delete rate_;
    delete kalmanFilter_;
    delete puckPredictor_;
    delete systemModel_;
    delete observationModel_;
    delete collisionModel_;
    delete visualizer_;
    delete particleFilter_;
    delete particleVisualizationInterface_;
}

void PuckTracker::start() {
    thread1_ = boost::thread(&PuckTracker::provideServices, this);
    thread_ = boost::thread(&PuckTracker::startTracking, this);
}

const PuckPredictedState &PuckTracker::getPredictedState(bool visualize, bool delayed) {
    getPrediction(predictedState_.predictedTime, predictedState_.numOfCollisions);
    predictedState_.state = puckPredictor_->getState();
    predictedState_.stamp = ros::Time::now();

    int bufferSize;
    if (delayed) {
        bufferSize = maxPredictionSteps_;
    } else {
        bufferSize = 0;
    }

    stateBuffer_.push_back(predictedState_.state);
    if (stateBuffer_.size() > bufferSize) {
        if (visualize) {
            puckPredictor_->updateInnovationCovariance(*observationModel_);
            visualizer_->update(stateBuffer_.front(),
                                puckPredictor_->getInnovationCovariance(),
                                true);
        }
        stateBuffer_.erase(stateBuffer_.begin());
    }
    validation_->publishPrediction(predictedState_.state, predictedState_.stamp, predictedState_.predictedTime);
    return predictedState_;
}

const PuckState &PuckTracker::getEstimatedState(bool visualize) {
    if (visualize) {
        kalmanFilter_->updateInnovationCovariance(*observationModel_);
        visualizer_->update(kalmanFilter_->getState(),
                            kalmanFilter_->getInnovationCovariance(),
                            false);
    }
    return kalmanFilter_->getState();
}

void PuckTracker::loadParams() {
    ROS_INFO_STREAM("Read Air Hockey Parameters");
    double malletRadius, puckRadius;
    double tableLength_, tableWidth_, goalWidth_;
    nh_.param<double>("/air_hockey/mallet_radius", malletRadius, 0.04815);
    nh_.param<double>("/air_hockey/puck_radius", puckRadius, 0.03165);
    nh_.param<double>("/air_hockey/table_length", tableLength_, 1.956);
    nh_.param<double>("/air_hockey/table_width", tableWidth_, 1.042);
    nh_.param<double>("/air_hockey/goal_width", goalWidth_, 0.25);

    ROS_INFO_STREAM("Read Puck Tracker Parameters");
    double frequency, frictionDrag, frictionSliding, restitutionTable, restitutionMallet;
    nh_.param<double>("/air_hockey/puck_tracker/friction_drag", frictionDrag, 0.1);
    nh_.param<double>("/air_hockey/puck_tracker/friction_sliding", frictionSliding, 0.0);
    nh_.param<double>("/air_hockey/puck_tracker/restitution_table", restitutionTable, 0.8);
    nh_.param<double>("/air_hockey/puck_tracker/restitution_mallet", restitutionMallet, 0.1);
    nh_.param<int>("/air_hockey/puck_tracker/max_prediction_steps", maxPredictionSteps_, 20);
    nh_.param<double>("/air_hockey/puck_tracker/frequency", frequency, 120.0);
    nh_.param<bool>("/air_hockey/puck_tracker/turn_on_pf", turn_on_pf, false);

    if (nh_.getNamespace() == "/iiwa_back") {
        tableRefName_ = "TableAway";
        opponentMalletName_ = "F_striker_mallet_tip";
    } else if (nh_.getNamespace() == "/iiwa_front") {
        tableRefName_ = "TableHome";
        opponentMalletName_ = "B_striker_mallet_tip";
    } else {
        ROS_ERROR_STREAM("No namespace specified for the puck tracker, use /iiwa_front or /iiwa_back");
        exit(-1);
    }

    ROS_INFO_STREAM("Wait for transform: /Table");
    ros::Duration(1.0).sleep();
    try {
        tfBuffer_.lookupTransform("world", "Table", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &exception) {
        ROS_ERROR_STREAM("Could not transform world to Table: " << exception.what());
        exit(-1);
    }

    rate_ = new ros::Rate(frequency);
    systemModel_ = new SystemModel(frictionDrag, frictionSliding);
    observationModel_ = new ObservationModel;
    collisionModel_ = new CollisionModel(tableLength_, tableWidth_, goalWidth_, puckRadius, malletRadius,
                                         restitutionTable, restitutionMallet, rate_->expectedCycleTime().toSec());
    kalmanFilter_ = new EKF_Wrapper;
    puckPredictor_ = new EKF_Wrapper;
    visualizer_ = new VisualizationInterface(nh_, tableRefName_);
    particleFilter_ = new ParticleFilter(*systemModel_);
    particleVisualizationInterface_ = new ParticleVisualizationInterface(nh_);
    validation_ = new ValidationInterface(nh_);

    //! Initialize Kalman Filter
    PuckState sInit;
    sInit.setZero();
    kalmanFilter_->init(sInit);
    setCovariance();

    u_.dt() = rate_->expectedCycleTime().toSec();

    doPrediction_ = true;
    useParticleFilter_ = false;
}

void PuckTracker::setCovariance() {
    Kalman::Covariance<PuckState> covDyn;
    Kalman::Covariance<Measurement> covObs;

    covObs.setIdentity();
    covDyn.setIdentity();

    double obsVarPos, obsVarAng, dynVarPos, dynVarVel, dynVarAngPos, dynVarAngVel;
    nh_.param<double>("/air_hockey/puck_tracker/observation_variance_position", obsVarPos, 1e-6);
    nh_.param<double>("/air_hockey/puck_tracker/observation_variance_angular", obsVarAng, 1e-6);
    nh_.param<double>("/air_hockey/puck_tracker/dynamic_variance_position", dynVarPos, 1e-4);
    nh_.param<double>("/air_hockey/puck_tracker/dynamic_variance_velocity", dynVarVel, 1e-2);
    nh_.param<double>("/air_hockey/puck_tracker/dynamic_variance_angular_position", dynVarAngPos, 1e-4);
    nh_.param<double>("/air_hockey/puck_tracker/dynamic_variance_angular_velocity", dynVarAngVel, 1e-2);
    nh_.param<double>("/air_hockey/puck_tracker/reset_threshold", resetThreshold, 6.25);
    ROS_INFO_STREAM("Observer Variance Position: " << obsVarPos);
    ROS_INFO_STREAM("Observer Variance Angular Position: " << obsVarAng);
    ROS_INFO_STREAM("Dynamics Variance Position:" << dynVarPos);
    ROS_INFO_STREAM("Dynamics Variance Velocity: " << dynVarVel);
    ROS_INFO_STREAM("Dynamics Variance Angular Position:" << dynVarAngPos);
    ROS_INFO_STREAM("Dynamics Variance Angular Velocity: " << dynVarAngVel);
    ROS_INFO_STREAM("Reset Threshold: " << resetThreshold);

    covDyn(0, 0) = dynVarPos;
    covDyn(1, 1) = dynVarPos;
    covDyn(2, 2) = dynVarVel;
    covDyn(3, 3) = dynVarVel;
    covDyn(4, 4) = dynVarAngPos;
    covDyn(5, 5) = dynVarAngVel;

    covObs(0, 0) = obsVarPos;
    covObs(1, 1) = obsVarPos;
    covObs(2, 2) = obsVarAng;

    systemModel_->setCovariance(covDyn);
    observationModel_->setCovariance(covObs);
    kalmanFilter_->setCovariance(covDyn);
}

void PuckTracker::startTracking() {
    rate_->sleep();
    stamp_ = ros::Time::now();
    while (nh_.ok()) {
        rate_->sleep();
        if (doPrediction_) {
            //! predict step
            kalmanFilter_->predict(*systemModel_, u_);
            if( collisionModel_->applyCollision(kalmanFilter_->getState(), true)){
                useParticleFilter_ = true;
            }
            if (useParticleFilter_) {
                particleFilter_->sampleParticles(kalmanFilter_->getState(),
                                                 const_cast<Eigen::Matrix<double, 6, 6> &>(kalmanFilter_->getCovariance()));
                particleVisualizationInterface_->visualize(*particleFilter_, tableRefName_);
            }
        }

        //! Update puck state
        if (getMeasurement()) {
            updateOpponentMallet();

            //! Check if puck measurement outside the table range, if not, update.
            if (!collisionModel_->m_table.isOutsideBoundary(measurement_)) {
                doPrediction_ = true;
                if (checkGating()) {
                    ROS_INFO_STREAM("[Puck Tracker] Update Kalman Filter");
                    if (useParticleFilter_ && turn_on_pf) {
                        kalmanFilter_->setCovariance(particleFilter_->applyParticleFilter(u_));
                    } else {
                        kalmanFilter_->update(*observationModel_, measurement_);
                    }
                } else {
                    ROS_INFO_STREAM("[Puck Tracker] The innovation is too big, reset the puck tracker");
                    reset();
                    continue;
                }
            } else {
                doPrediction_ = false;
            }
        }else{
            ROS_INFO_STREAM("No measurement - no prediction");
        }
        stamp_ += rate_->expectedCycleTime();
    }
}

bool PuckTracker::getMeasurement() {
    try {
        tfPuck_ = tfBuffer_.lookupTransform(tableRefName_, "Puck", ros::Time(0));
        if ((tfPuck_.header.stamp - stamp_) > rate_->expectedCycleTime()) {
            stamp_ = tfPuck_.header.stamp;
        } else if (tfPuck_.header.stamp - stamp_ < -rate_->expectedCycleTime()) {
//		    ROS_WARN_STREAM("TF data is " << tfPuck_.header.stamp - stamp_<< "s behind, Ignored");
            return false;
        }
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
        return true;
    } catch (tf2::TransformException &exception) {
        //ROS_WARN_STREAM("Transformation table to puck failed: " << exception.what());
        return false;
    }
}

bool PuckTracker::updateOpponentMallet() {
    try {
        tfOpponentMallet_ = tfBuffer_.lookupTransform(tableRefName_, opponentMalletName_, ros::Time(0));
        collisionModel_->m_mallet.setState(tfOpponentMallet_);
        return true;
    } catch (tf2::TransformException &exception) {
        return false;
    }
}

bool PuckTracker::checkGating() {
    auto InnoCov = kalmanFilter_->updateInnovationCovariance(*observationModel_);
    kalmanFilter_->calculateInnovation(*observationModel_, measurement_);
    double ellipsoidalDist = kalmanFilter_->getInnovation().block<2, 1>(0, 0).transpose() *
                             InnoCov.block<2, 2>(0, 0).inverse() *
                             kalmanFilter_->getInnovation().block<2, 1>(0, 0);
    return ellipsoidalDist < resetThreshold;
}

void PuckTracker::getPrediction(double &predictedTime, int &nCollision) {
    nCollision = 0;
    predictedTime = 0;
    if (doPrediction_) {
        //! predict future steps
        puckPredictor_->init(kalmanFilter_->getState());
        puckPredictor_->setCovariance(kalmanFilter_->getCovariance());
        if (defendingLine_ <= 0.0) {
            //! If not consider defending line, do maximum steps prediction
            for (int i = 0; i < maxPredictionSteps_; ++i) {
                puckPredictor_->moveOneStep(*systemModel_, u_);
                if (collisionModel_->applyCollision(puckPredictor_->getState(), true)) {
                    nCollision += 1;
                    useParticleFilter_ = true;
                }else{
                    useParticleFilter_ = false;
                }
                predictedTime = (i + 1) * rate_->expectedCycleTime().toSec();
                if(i == 0){
                    validation_->publishOneStepPrediction(puckPredictor_->getState(), predictedTime);
                }
            }
        } else {
            //! If consider defending line
            bool should_defend = (puckPredictor_->getState().x() > defendingLine_);
            for (int i = 0; i < maxPredictionSteps_; ++i) {
                if (should_defend && puckPredictor_->getState().x() < defendingLine_) {
                    return;
                }
                puckPredictor_->moveOneStep(*systemModel_, u_);
                if (collisionModel_->applyCollision(puckPredictor_->getState(), true)) {
                    nCollision += 1;
                    useParticleFilter_ = true;
                }else{
                    useParticleFilter_ = false;
                }
                predictedTime = (i + 1) * rate_->expectedCycleTime().toSec();
            }
        }
    }
}

void PuckTracker::reset() {
    try {
        geometry_msgs::TransformStamped tf1, tf2;
        //! Look up for the second frame
        stamp_ = ros::Time::now();
        tf1 = tfBuffer_.lookupTransform(tableRefName_, "Puck", stamp_, ros::Duration(0.02));
        tf2::Quaternion quat;
        quat.setX(tf1.transform.rotation.x);
        quat.setY(tf1.transform.rotation.y);
        quat.setZ(tf1.transform.rotation.z);
        quat.setW(tf1.transform.rotation.w);
        tf2::Matrix3x3 rotMat(quat);
        double roll1, pitch1, yaw1;
        rotMat.getEulerYPR(yaw1, pitch1, roll1);

        stamp_ += rate_->expectedCycleTime() * 10;
        tf2 = tfBuffer_.lookupTransform(tableRefName_, "Puck", stamp_, ros::Duration(1.0));
        double dt = (tf2.header.stamp - tf1.header.stamp).toSec();
        quat.setX(tf2.transform.rotation.x);
        quat.setY(tf2.transform.rotation.y);
        quat.setZ(tf2.transform.rotation.z);
        quat.setW(tf2.transform.rotation.w);
        rotMat.setRotation(quat);
        double roll2, pitch2, yaw2;
        rotMat.getEulerYPR(yaw2, pitch2, roll2);

        //! set Initial State
        PuckState initState;
        initState.dx() = (tf2.transform.translation.x - tf1.transform.translation.x) / dt;
        initState.dy() = (tf2.transform.translation.y - tf1.transform.translation.y) / dt;
        initState.x() = tf2.transform.translation.x;
        initState.y() = tf2.transform.translation.y;

        initState.dtheta() = tf2NormalizeAngle((yaw2 - yaw1) / dt);
        initState.theta() = yaw2;

        kalmanFilter_->init(initState);

        Kalman::Covariance<PuckState> covInit;
        covInit.setIdentity();
        kalmanFilter_->setCovariance(covInit);

        puckPredictor_->init(initState);
        puckPredictor_->setCovariance(covInit);

    } catch (tf2::TransformException &exception) {
        ROS_INFO_STREAM("Reset Failed: " << exception.what() << " Kill Puck Tracker");
        exit(-1);
    }
}

void PuckTracker::publishData(const PuckState &prediction, const PuckState &measurement) {
    validation_->record(prediction, measurement);
}

void PuckTracker::provideServices(){
    ros::ServiceServer dynamicsService = nh_.advertiseService("set_dynamics_parameter", &PuckTracker::setDynamicsParameter, this);
    ros::ServiceServer resetService = nh_.advertiseService("reset_puck_tracker", &PuckTracker::resetService, this);
    ROS_INFO_STREAM("Services ready to call");
    ros::spin();
}


bool PuckTracker::setDynamicsParameter(air_hockey_puck_tracker::SetDynamicsParameter::Request &req,
                         air_hockey_puck_tracker::SetDynamicsParameter::Response &res){
    ROS_INFO_STREAM("Setting dynamics parameter to Damping:" << req.frictionDrag << " mu: " << req.frictionSlide);
    systemModel_->setDamping(req.frictionDrag);
    systemModel_->setMu(req.frictionSlide);

    collisionModel_->setMalletRestitution(req.restitutionMallet);
    collisionModel_->setTableRestitution(req.restitutionTable);
    res.success = true;
    return true;
}

bool PuckTracker::resetService(air_hockey_puck_tracker::PuckTrackerResetService::Request &req,
                               air_hockey_puck_tracker::PuckTrackerResetService::Response &res){
    ROS_INFO_STREAM("Setting initial Puck State via Service");
    //! set Initial State
    PuckState initState;
    initState.dx() = req.dx;
    initState.dy() = req.dy;
    initState.x() = req.x;
    initState.y() = req.y;

    initState.dtheta() = req.dtheta;
    initState.theta() = req.theta;

    kalmanFilter_->init(initState);

    Kalman::Covariance<PuckState> covInit;
    covInit.setIdentity();
    kalmanFilter_->setCovariance(covInit);

    puckPredictor_->init(initState);
    puckPredictor_->setCovariance(covInit);

    res.success = true;
    return true;
}
