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

#ifndef PUCK_TRACKER_VALIDATION_HPP
#define PUCK_TRACKER_VALIDATION_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ros/package.h>
#include <vector>
#include "SystemModel.hpp"
#include "ObservationModel.hpp"
#include "puck_tracker/InnovationMsg.h"


namespace AirHockey{
    class ValidationInterface{
        typedef Kalman::Covariance<Measurement> InnovationCovariance;

    public:
        ValidationInterface(ros::NodeHandle nh_, bool save=false):m_nh(nh_){
            m_pub = m_nh.advertise<puck_tracker::InnovationMsg>("innovation", 1000);
            m_save = save;
            if (m_save) {
                string path = ros::package::getPath("puck_tracker");
                stringstream filepath;
                auto t = std::time(nullptr);
                auto tm = *std::localtime(&t);
                filepath << path << "/data/" << std::put_time(&tm, "%Y-%d-%m-%H-%M-%S") << ".bag";

                bag.open(filepath.str(), rosbag::bagmode::Write);
            }

        }

        ~ValidationInterface(){
            bag.close();
        }

        void record(const Measurement& mu, const InnovationCovariance& S){
            m_msg.header.stamp = ros::Time::now();
            m_msg.size = 1;

            m_msg.x = mu.x();
            m_msg.y = mu.y();

            m_msg.normalized = mu.transpose() * S.inverse() * mu;
            m_pub.publish(m_msg);
            if (m_save){
                bag.write("innovation", ros::Time::now(), m_msg);
            }
        }


    public:
        ros::NodeHandle m_nh;
        rosbag::Bag bag;
        ros::Publisher m_pub;
        puck_tracker::InnovationMsg m_msg;

    protected:
        bool m_save;
    };
}

#endif //PUCK_TRACKER_VALIDATION_HPP
