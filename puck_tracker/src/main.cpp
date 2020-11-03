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
#include "CollisionModel.hpp"
#include "Validation.hpp"


using namespace AirHockey;

int main(int argc, char **argv) {
    ros::init(argc, argv, "puck_tracker");
    ros::NodeHandle nh("~");
    tf::TransformListener listener;
    ros::Rate rate(100);

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
    covDyn(4, 4) = 0.01;

    covObs(0, 0) = 0.0001;
    covObs(1, 1) = 0.0001;

    dynamicsModel.setCovariance(covDyn);
    observationModel.setCovariance(covObs);
    ekf.setCovariance(covInit);

    //! Initialize State
    Control u;
    State sInit;
    sInit.setZero();
    sInit(4) = 0.05;
    tf::StampedTransform tfTable, tfPrev;
    ROS_INFO_STREAM("Wait for the TF message");

    if (listener.waitForTransform("/Table", "/world", ros::Time(0), ros::Duration(30))) {
        listener.lookupTransform("/world", "/Table", ros::Time(0), tfTable);
        listener.lookupTransform("/world", "/Table", ros::Time(0), tfPrev);
        sInit.x() = tfPrev.getOrigin().x();
        sInit.y() = tfPrev.getOrigin().y();
    } else {
        ROS_ERROR_STREAM("Cannot find TF of Air Hockey Table!");
        return 0;
    }

    ekf.init(sInit);

    //! Initialize visualization table height
    VisualizationInterface visualizationInterface(nh, tfTable.getOrigin().z());

    //! Initialize validation
    ValidationInterface validationInterface(nh, false);

    //! Collision Model
    AirHockeyTable table(tfTable);
    CollisionModel collisionModel(table);
    collisionModel.m_dt = rate.expectedCycleTime().toSec();

    ROS_INFO_STREAM("Start Tracking.");

    Measurement measurementTmp;

    predictor.init(ekf.getState());
    predictor.setCovariance(ekf.getCovariance());

    u.dt() = rate.expectedCycleTime().toSec();

    int predict_steps = 20;
    std::vector<Measurement> predictionBuffer;
    std::vector<EKF::InnovationCovariance> innovationCovBuffer;
    while (ros::ok()) {
        //! predict step
        auto x_ekf = ekf.predict(dynamicsModel, u);
        collisionModel.applyCollision(ekf.getState());
        ekf.updateInnovationCovariance(observationModel);

        //! predict future step
        predictor.init(ekf.getState());
        predictor.setCovariance(ekf.getCovariance());
        for (int i = 0; i < predict_steps; ++i) {
            predictor.predict(dynamicsModel, u);
            predictor.updateInnovationCovariance(observationModel);
            collisionModel.applyCollision(predictor.getState());
        }

        //! update buffer
        if (predictionBuffer.size() > predict_steps || innovationCovBuffer.size() > predict_steps){
            predictionBuffer.erase(predictionBuffer.begin());
            innovationCovBuffer.erase(innovationCovBuffer.begin());
        }
        predictionBuffer.push_back(observationModel.h(predictor.getState()));
        innovationCovBuffer.push_back(predictor.getInnovationCovariance());

        visualizationInterface.setPredictionMarker(predictor.getState(), predictor.getInnovationCovariance());
        visualizationInterface.visualize();

        //! Update step
        tf::StampedTransform tfTmp;
        auto time = ros::Time::now();
        if (listener.waitForTransform("/world", "/Puck", time, rate.expectedCycleTime() - rate.cycleTime())) {
            listener.lookupTransform("/world", "/Puck", time, tfTmp);

            measurementTmp.x() = tfTmp.getOrigin().x();
            measurementTmp.y() = tfTmp.getOrigin().y();

            x_ekf = ekf.update(observationModel, measurementTmp);

            tfPrev = tfTmp;

            validationInterface.record(measurementTmp - predictionBuffer[0], innovationCovBuffer[0]);
        }

        rate.sleep();
    }
    return 0;
}

