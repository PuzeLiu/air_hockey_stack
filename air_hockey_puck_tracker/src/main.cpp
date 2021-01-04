/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
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

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "air_hockey_puck_tracker/CollisionModel.hpp"
#include "air_hockey_puck_tracker/EKF_Wrapper.hpp"
#include "air_hockey_puck_tracker/ObservationModel.hpp"
#include "air_hockey_puck_tracker/VisualizationInterface.hpp"
#include "air_hockey_puck_tracker/Validation.hpp"


using namespace AirHockey;
using namespace std;

void setCovariance(ros::NodeHandle &nh, Kalman::Covariance<State> &covDyn, Kalman::Covariance<Measurement> &covObs,
                   Kalman::Covariance<State> &covInit) {
    covObs.setIdentity();
    covDyn.setIdentity();
    covInit.setIdentity();

    double obsVarPos, obsVarAng, dynVarPos, dynVarVel, dynVarAngPos, dynVarAngVel;
    nh.param<double>("observation_variance_position", obsVarPos, 1e-6);
    nh.param<double>("observation_variance_angular", obsVarAng, 1e-6);
    nh.param<double>("dynamic_variance_position", dynVarPos, 1e-4);
    nh.param<double>("dynamic_variance_velocity", dynVarVel, 1e-2);
    nh.param<double>("dynamic_variance_angular_position", dynVarAngPos, 1e-4);
    nh.param<double>("dynamic_variance_angular_velocity", dynVarAngVel, 1e-2);
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
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "puck_tracker");
    ros::NodeHandle nh("~");
    tf::TransformListener listener;
    ros::Rate rate(120);

    ROS_INFO_STREAM("Read System Parameters");
    int predict_steps;
    double frictionDrag, frictionSliding, restitutionTable, restitutionMallet;
    nh.param<double>("friction_drag", frictionDrag, 0.1);
    nh.param<double>("friction_sliding", frictionSliding, 0.0);
    nh.param<double>("restitution_table", restitutionTable, 0.8);
    nh.param<double>("restitution_mallet", restitutionMallet, 0.1);
    nh.param<int>("prediction_steps", predict_steps, 20);

    ROS_INFO_STREAM("Drag Parameter:" << frictionDrag);
    ROS_INFO_STREAM("Sliding Parameter: " << frictionSliding);
    ROS_INFO_STREAM("Restitution Parameter of Wall: " << restitutionTable);
    ROS_INFO_STREAM("Restitution Parameter of Mallet: " << restitutionMallet);
    ROS_INFO_STREAM("Prediction Steps: " << predict_steps);

    ROS_INFO_STREAM("Wait for the TF message: /Table");

    tf::StampedTransform tfTable, tfPrev;

    listener.waitForTransform("/world", "/Table", ros::Time(0), ros::Duration(30.0));
    listener.lookupTransform("/world", "/Table", ros::Time(0), tfTable);

    //! Kalman Filter Model
    SystemModel dynamicsModel(frictionDrag, frictionSliding);
    ObservationModel observationModel;

    //! Collision Model
    CollisionModel collisionModel(tfTable, restitutionTable, restitutionMallet, rate.expectedCycleTime().toSec());

    //! Initialize Covariance Matrix
    Kalman::Covariance<State> covDyn;
    Kalman::Covariance<Measurement> covObs;
    Kalman::Covariance<State> covInit;
    setCovariance(nh, covDyn, covObs, covInit);
    dynamicsModel.setCovariance(covDyn);
    observationModel.setCovariance(covObs);

    //! Initialize State and Control Vector
    ROS_INFO_STREAM("Wait for the TF message: /Puck");
    listener.waitForTransform("/world", "/Puck", ros::Time(0), ros::Duration(30.0));
    listener.lookupTransform("/world", "/Puck", ros::Time(0), tfPrev);
    State sInit;
    sInit.setZero();
    sInit.x() = tfPrev.getOrigin().x();
    sInit.y() = tfPrev.getOrigin().y();

    Control u;
    u.dt() = rate.expectedCycleTime().toSec();

    //! Initialize Kalman Filter
    EKF_Wrapper ekf;
    EKF_Wrapper predictor;
    ekf.init(sInit);
    ekf.setCovariance(covInit);
    predictor.init(ekf.getState());
    predictor.setCovariance(ekf.getCovariance());

    //! Initialize visualization table height
    VisualizationInterface visualizationInterface(nh, tfTable.getOrigin().z());

    //! Initialize validation
    ValidationInterface validationInterface(nh, false);

    Measurement measurementTmp;

    std::vector<Measurement> predictionBuffer;
    std::vector<EKF_Wrapper::InnovationCovariance> innovationCovBuffer;

    ROS_INFO_STREAM("Start Tracking.");
    bool reInit = true;
    while (ros::ok()) {
        if (reInit){
            int updateCount = 0;
            ekf.setCovariance(covInit);
            predictionBuffer.clear();
            innovationCovBuffer.clear();
            while (updateCount < 10){
                auto time = ros::Time::now();
                ekf.predict(dynamicsModel, u);
                collisionModel.m_table.applyCollision(ekf.getState());
                if (listener.waitForTransform("/world", "/Puck", time, rate.expectedCycleTime())) {
                    tf::StampedTransform tfTmp;
                    listener.lookupTransform("/world", "/Puck", time, tfTmp);
                    double thetaTmp = tf::getYaw(tfTmp.getRotation());
                    measurementTmp.x() = tfTmp.getOrigin().x();
                    measurementTmp.y() = tfTmp.getOrigin().y();
                    measurementTmp.theta() = thetaTmp;

                    ekf.update(observationModel, measurementTmp);
                    tfPrev = tfTmp;
                    updateCount ++;
                }
            }
            reInit = false;
        }
        else {
            auto time = ros::Time::now();
            if (listener.waitForTransform("/world", "/Mallet", time, rate.expectedCycleTime())) {
                tf::StampedTransform tfTmp;
                listener.lookupTransform("/world", "/Mallet", time, tfTmp);
                collisionModel.m_mallet.setState(tfTmp);
            }

            //! predict step
            u.dt() = rate.cycleTime().toSec();
            ekf.predict(dynamicsModel, u);
            collisionModel.m_table.applyCollision(ekf.getState());
            if (collisionModel.m_mallet.applyCollision(ekf.getState())) {
                reInit = true;
                continue;
            }

            //! predict future step
            predictor.init(ekf.getState());
            predictor.setCovariance(ekf.getCovariance());
            u.dt() = rate.expectedCycleTime().toSec();
            for (int i = 0; i < predict_steps; ++i) {
                predictor.predict(dynamicsModel, u);
                collisionModel.m_mallet.predict();
                collisionModel.m_table.applyCollision(predictor.getState());
            }

            visualizationInterface.setPredictionMarker(predictor.getState(),
                                                       predictor.updateInnovationCovariance(observationModel));
            visualizationInterface.visualize();

            predictionBuffer.push_back(observationModel.h(predictor.getState()));
            innovationCovBuffer.push_back(predictor.getInnovationCovariance());

            //! Update step
            time = ros::Time::now();
            if (listener.waitForTransform("/world", "/Puck", time, rate.expectedCycleTime())) {
                tf::StampedTransform tfTmp;
                listener.lookupTransform("/world", "/Puck", time, tfTmp);

                double thetaTmp = tf::getYaw(tfTmp.getRotation());

                measurementTmp.x() = tfTmp.getOrigin().x();
                measurementTmp.y() = tfTmp.getOrigin().y();
                measurementTmp.theta() = thetaTmp;

                ekf.update(observationModel, measurementTmp);

                tfPrev = tfTmp;
            }

            //! update buffer
            if (predictionBuffer.size() > predict_steps && innovationCovBuffer.size() > predict_steps) {
                validationInterface.record(predictionBuffer[0], measurementTmp, innovationCovBuffer[0]);
                predictionBuffer.erase(predictionBuffer.begin());
                innovationCovBuffer.erase(innovationCovBuffer.begin());
            }

        }
        rate.sleep();
    }
    return 0;
}

