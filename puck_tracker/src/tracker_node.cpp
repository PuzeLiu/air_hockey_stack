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
#include "ekf.h"
#include "ros_visualization.h"
#include <mrpt/math/CMatrixDynamic.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "puck_tracker");
    ros::NodeHandle nh("~");
    ros::Rate rate(100);

    DynamicsParam dynaParam = {0.0, 0.0,};
    ModelNoise modelNoise = {0.5, 1.0, 1.0, 1.0};

    EKF ekf_dynamics(dynaParam, modelNoise, rate.cycleTime().toSec());

    VisualizationInterface visualizationInterface(nh);



    mrpt::math::CVectorDynamic<double> xkk(4);
    mrpt::math::CMatrixDynamic<double> pkk(4);

    ekf_dynamics.init();
    while (ros::ok()){
        ekf_dynamics.doProcess();
        ekf_dynamics.getState(xkk, pkk);
        visualizationInterface.setPredictionMarker(xkk, pkk);
        visualizationInterface.visualize();

        rate.sleep();
    }
    return 0;
}
