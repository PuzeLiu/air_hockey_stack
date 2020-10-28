/*
 * MIT License
 * Copyright (c) 2020 Davide Tateo, Puze Liu
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
#include "RosVisualization.hpp"
#include "ObservationModel.hpp"
#include "EKF.hpp"

typedef double T;

using namespace AirHockey;

State initializeState(const tf::TransformListener& listener, tf::StampedTransform& tfPrev){
    bool initialized = false;
    State x0;

    while (!initialized){
        try{
            listener.lookupTransform("/world", "/Puck", ros::Time(0), tfPrev);
            x0.x() = tfPrev.getOrigin().x();
            x0.y() = tfPrev.getOrigin().y();
            x0.dx() = 0.;
            x0.dy() = 0.;
            x0.c() = 0.1;
            break;
        }
        catch (tf::TransformException ex) {
            ROS_WARN_ONCE("Wait tf for initialziation");
        }
    }
    return x0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "puck_tracker");
    ros::NodeHandle nh("~");
    tf::TransformListener listener;

    VisualizationInterface visualizationInterface(nh);

    SystemModel dynamicsModel;
    ObservationModel observationModel;
    EKF ekf;
    EKF predictor;

    // Initialize Covariance Matrix
    Kalman::Covariance<State> covDyn;
    Kalman::Covariance<Measurement> covObs;
    Kalman::Covariance<State> covInit;

    covDyn.setIdentity();
    covObs.setIdentity();
    covInit.setIdentity();

    covDyn(0, 0) = 0.0001;
    covDyn(1, 1) = 0.0001;
    covDyn(2, 2) = 0.01;
    covDyn(3, 3) = 0.01;
    covDyn(4, 4) = 0.;

    covObs(0, 0) = 0.000009;
    covObs(1, 1) = 0.000009;

    dynamicsModel.setCovariance(covDyn);
    observationModel.setCovariance(covObs);
    ekf.setCovariance(covInit);

    tf::StampedTransform tfPrev;

    // Initialize State
    State x0 = initializeState(listener, tfPrev);
    ekf.init(x0);
    Control u;

    ROS_INFO_STREAM("Start Tracking.");

    Measurement measurementTmp;

    predictor.init(ekf.getState());
    predictor.setCovariance(ekf.getCovariance());

    ros::Rate rate(100);
    u.dt() = rate.expectedCycleTime().toSec();
    while (ros::ok()){
        auto x_ekf = ekf.predict(dynamicsModel, u);
        predictor.init(ekf.getState());
        predictor.setCovariance(ekf.getCovariance());

        ekf.updateInnovationCovariance(observationModel);


        tf::StampedTransform tfTmp;
        auto time = ros::Time::now();
        if (listener.waitForTransform("/world", "/Puck", time, rate.cycleTime()))
        {
            listener.lookupTransform("/world", "/Puck", time, tfTmp);

            measurementTmp.x() = tfTmp.getOrigin().x();
            measurementTmp.y() = tfTmp.getOrigin().y();

            x_ekf = ekf.update(observationModel, measurementTmp);

            tfPrev = tfTmp;
        }

        for (int i = 0; i < 20; ++i) {
            predictor.predict(dynamicsModel, u);
            predictor.updateInnovationCovariance(observationModel);
        }

//        ROS_INFO_STREAM("Observation: x: " << measurementTmp.x() << "  y: " << measurementTmp.y()<<
//                                           " dx: " << measurementTmp.dx() << " dy: " << measurementTmp.dy());
//        ROS_INFO_STREAM("Prediction : x: " << predictor.getState().x() << "  y: " << predictor.getState().y() <<
//                                           " dx: " << predictor.getState().dx() << " dy: " << predictor.getState().dy());
//        ROS_INFO_STREAM("             c: " << predictor.getState().c());

        visualizationInterface.setPredictionMarker(predictor.getState(), predictor.getInnovationCovariance());
        visualizationInterface.visualize();

        rate.sleep();
    }
    return 0;
}

